//! Nodal analysis of the passive crossover schematic.
//!
//! Nodes are grid coordinates: parts touching the same point share a node, a `Wire` joins all
//! its points, a `Ground` ties its point to node 0. Every two-terminal part uses its first two
//! points. The generator is a source `Eg·e^{−jωTg}` behind `Rg` (Norton form).

use crate::project::{Part, PartKind, Project};
use num_complex::Complex64;
use std::collections::HashMap;
use std::f64::consts::PI;

type C = Complex64;

/// Result of the circuit simulation on `freq`.
#[derive(Clone, Debug)]
pub struct Solution {
    pub freq: Vec<f64>,
    /// Load impedance seen by the generator (`Eg`/`Rg` included in neither: terminal voltage / delivered current).
    pub generator_z: Vec<C>,
    /// Voltage across each `Driver` part (+ minus −), in schematic order, referred to `Eg` = 1 V.
    pub driver_v: Vec<Vec<C>>,
    /// Indices into `Project::parts` of the drivers.
    pub driver_parts: Vec<usize>,
}

fn find(parent: &mut Vec<usize>, i: usize) -> usize {
    let mut r = i;
    while parent[r] != r {
        r = parent[r];
    }
    let mut i = i;
    while parent[i] != r {
        let next = parent[i];
        parent[i] = r;
        i = next;
    }
    r
}

/// Series impedance of a passive two-terminal part; `None` = open; a short is 1 nΩ.
fn passive_z(part: &Part, w: f64) -> Option<C> {
    if part.open {
        return None;
    }
    if part.shorted {
        return Some(C::new(1e-9, 0.0));
    }
    let s = C::new(0.0, w);
    Some(match part.kind {
        PartKind::Resistor => C::new(part.param("R")?, 0.0),
        PartKind::Capacitor => C::new(part.param("ESR").unwrap_or(0.0), 0.0) + 1.0 / (s * (part.param("C")? * 1e-6)),
        PartKind::Inductor => {
            // (DCR + sL) ∥ Rpar ∥ 1/(sCpar); Rpar and Cpar only when given
            let mut y = 1.0 / (C::new(part.param("DCR").unwrap_or(0.0), 0.0) + s * (part.param("L")? * 1e-3));
            if let Some(rp) = part.param("Rpar").filter(|&r| r > 0.0) {
                y += 1.0 / rp;
            }
            if let Some(cp) = part.param("Cpar").filter(|&c| c > 0.0) {
                y += s * (cp * 1e-6);
            }
            1.0 / y
        }
        _ => return None,
    })
}

/// Solves the schematic. `driver_z(part, f)` gives the impedance of a `Driver` part.
pub fn solve(p: &Project, freq: &[f64], driver_z: &dyn Fn(&Part, f64) -> C) -> Result<Solution, String> {
    // --- netlist ---
    let mut ids: HashMap<(i32, i32), usize> = HashMap::new();
    let mut parent = vec![0usize]; // 0 = ground
    let mut id_of = |pt: (i32, i32), parent: &mut Vec<usize>| {
        *ids.entry(pt).or_insert_with(|| {
            parent.push(parent.len());
            parent.len() - 1
        })
    };
    let mut terms: Vec<Option<(usize, usize)>> = vec![None; p.parts.len()];
    for (k, part) in p.parts.iter().enumerate() {
        let pts: Vec<usize> = part.points.iter().map(|&q| id_of(q, &mut parent)).collect();
        match part.kind {
            PartKind::Wire => {
                for w in pts.windows(2) {
                    let (a, b) = (find(&mut parent, w[0]), find(&mut parent, w[1]));
                    parent[a] = b;
                }
            }
            PartKind::Ground => {
                if let Some(&a) = pts.first() {
                    let (a, b) = (find(&mut parent, a), find(&mut parent, 0));
                    parent[a] = b;
                }
            }
            PartKind::Other => return Err(format!("unsupported part {} ({})", part.id, part.type_name)),
            _ => {
                if pts.len() < 2 {
                    return Err(format!("part {} has fewer than two terminals", part.id));
                }
                terms[k] = Some((pts[0], pts[1]));
            }
        }
    }
    // compact node numbering: ground = None
    let ground = find(&mut parent, 0);
    let mut index: HashMap<usize, usize> = HashMap::new();
    let mut node = |i: usize, parent: &mut Vec<usize>| -> Option<usize> {
        let r = find(parent, i);
        if r == ground {
            None
        } else {
            let n = index.len();
            Some(*index.entry(r).or_insert(n))
        }
    };
    let tn: Vec<Option<(Option<usize>, Option<usize>)>> =
        terms.iter().map(|t| t.map(|(a, b)| (node(a, &mut parent), node(b, &mut parent)))).collect();
    let n = index.len();

    let gen_k = p.parts.iter().position(|q| q.kind == PartKind::Generator).ok_or("no generator")?;
    let driver_parts: Vec<usize> = (0..p.parts.len()).filter(|&k| p.parts[k].kind == PartKind::Driver).collect();
    let gen = &p.parts[gen_k];
    let (eg, tg, rg) = (gen.param("Eg").unwrap_or(1.0), gen.param("Tg").unwrap_or(0.0) * 1e-6, gen.param("Rg").unwrap_or(0.001));
    let (g_plus, g_minus) = tn[gen_k].ok_or("generator terminals")?;

    let mut out = Solution { freq: freq.to_vec(), generator_z: vec![], driver_v: vec![vec![]; driver_parts.len()], driver_parts: driver_parts.clone() };
    for &f in freq {
        let w = 2.0 * PI * f;
        let mut y = vec![vec![C::new(0.0, 0.0); n]; n];
        let mut rhs = vec![C::new(0.0, 0.0); n];
        let stamp = |y: &mut Vec<Vec<C>>, a: Option<usize>, b: Option<usize>, g: C| {
            if let Some(a) = a {
                y[a][a] += g;
            }
            if let Some(b) = b {
                y[b][b] += g;
            }
            if let (Some(a), Some(b)) = (a, b) {
                y[a][b] -= g;
                y[b][a] -= g;
            }
        };
        for (k, part) in p.parts.iter().enumerate() {
            let Some((a, b)) = tn[k] else { continue };
            let z = match part.kind {
                PartKind::Generator => Some(C::new(rg, 0.0)),
                PartKind::Driver => Some(driver_z(part, f)),
                _ => passive_z(part, w),
            };
            if let Some(z) = z {
                stamp(&mut y, a, b, 1.0 / z);
            }
        }
        // generator: Norton current source Eg/Rg from − to +
        let vs = C::from_polar(eg, -w * tg);
        if let Some(a) = g_plus {
            rhs[a] += vs / rg;
        }
        if let Some(b) = g_minus {
            rhs[b] -= vs / rg;
        }
        let v = gauss(y, rhs).ok_or_else(|| format!("singular circuit at {f} Hz"))?;
        let at = |i: Option<usize>| i.map_or(C::new(0.0, 0.0), |i| v[i]);
        let vt = at(g_plus) - at(g_minus);
        out.generator_z.push(vt / ((vs - vt) / rg));
        for (j, &k) in driver_parts.iter().enumerate() {
            let (a, b) = tn[k].unwrap();
            out.driver_v[j].push((at(a) - at(b)) / eg);
        }
    }
    Ok(out)
}

/// Complex Gaussian elimination with partial pivoting.
fn gauss(mut a: Vec<Vec<C>>, mut b: Vec<C>) -> Option<Vec<C>> {
    let n = b.len();
    for c in 0..n {
        let piv = (c..n).max_by(|&i, &j| a[i][c].norm().total_cmp(&a[j][c].norm()))?;
        if a[piv][c].norm() < 1e-300 {
            return None;
        }
        a.swap(c, piv);
        b.swap(c, piv);
        for r in c + 1..n {
            let m = a[r][c] / a[c][c];
            if m.norm() == 0.0 {
                continue;
            }
            for k in c..n {
                let t = a[c][k];
                a[r][k] -= m * t;
            }
            let t = b[c];
            b[r] -= m * t;
        }
    }
    let mut x = vec![C::new(0.0, 0.0); n];
    for i in (0..n).rev() {
        let s: C = (i + 1..n).map(|k| a[i][k] * x[k]).sum();
        x[i] = (b[i] - s) / a[i][i];
    }
    Some(x)
}
