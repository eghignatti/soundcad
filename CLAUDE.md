# SoundCAD – stato

Simulatore integrato di altoparlanti (layout frontale, driver, cassa, crossover, risposta live). Rust + eframe/egui (wgpu), workspace con 5 crate + `webbrowser-stub`.

## Build e prova su Linux (8 nov 2026)
- `export CARGO_HOME=$PWD/.cargo-home; cargo build --release` → ok, ~25 s, nessun errore.
- Binario: `target/release/soundcad`. Avviato 8 s su Wayland/X11 (CachyOS): nessun crash, nessun output di log.
- `cargo test`: 0 test in tutti i crate (nessuna copertura automatica).
- Dati demo in `crates/soundcad/demo/` (file ZR e polari .txt).

## Da fare
- Verifica manuale dell'interfaccia (apertura/salvataggio file via portale XDG, import demo).
- Aggiungere test sui calcoli (core, xover, speaker).
- Ottimizzazione: da misurare prima (profilo del ricalcolo live).

## Lunghezza tubo e interno cassa (8 nov 2026)
- Lunghezza tubo reflex: ora anche nella sezione "Vent" del pannello driver (stesso valore `len` della scheda Box).
- Interno cassa (`soundcad-tools/src/cabinet.rs`, 5 test): `Unit.cabinet: Option<Cabinet>` (profondità interna, spessore pareti, spessore/densità/% rivestimento).
  Larghezza e altezza interne dal fronte. Superficie − fori (tagli di tutti i driver + tubi) = max rivestibile.
  Se attivo, Vb netto (lordo − rivestimento − tubi) e Qa sono calcolati e sovrascrivono quelli del Radiator in `Speaker::evaluate`
  (`cabinet_analysis`, `Analysis::fit`). Qa: formula empirica; assorbimento: Delany-Bazley (X limitato a 0,01…1), σ = 25,989·ρ^1,404 (Garai-Pompoli, poliestere); T60 Sabine.
- Modi interni (scatola rettangolare, ≤3 kHz): correzione complessa sulla pressione alla porta, applicata in `Enclosure.modes`
  (vale per tubo e radiatore passivo). Il cono non sente i modi. Stima, non validata su misure.
- Non ho visto l'interfaccia a schermo (solo avvio senza crash): da provare a mano.
- Letteratura (9 nov 2026): σ(ρ) Garai-Pompoli 2005 (costanti da poster Pelegrinis et al., SAPEM 2014, non dall'articolo originale);
  Delany-Bazley 1970 valido 0,01<X<1; smorzamento modi per tipo (assiale/tangenziale/obliquo), rapporto 1:1,25:1,5 verificato su COMSOL (Morse-Ingard).
  Non verificato in fonte: coefficienti Miki (non usati). Qa dal rivestimento: formula empirica senza fonte primaria
  (ancore: riempimento pesante ≈3, rivestimento moderato ≈10-30, da forum/Leach 1989).
- Fori: il taglio del tweeter conta nella superficie non rivestibile, ma non entra nell'acustica interna (solo il driver della cassa eccita i modi).
