use std::{error::Error, fs, path::Path};

use linkhub_clientgen::{load_profile, python_types, typescript_types};

const PYTHON_OUTPUT: &str = "../../linkhub_client/src/linkhub_client/generated_protocol.py";
const TYPESCRIPT_OUTPUT: &str = "../../linkhub-ui/src/generated/protocol.ts";

fn main() -> Result<(), Box<dyn Error>> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let profile = load_profile(&root.join("../dialect/definitions"))?;
    for (output, text) in [
        (PYTHON_OUTPUT, python_types(&profile)),
        (TYPESCRIPT_OUTPUT, typescript_types(&profile)),
    ] {
        let path = root.join(output);
        fs::write(&path, text)?;
        println!("wrote {}", path.display());
    }
    Ok(())
}
