// napi-rs build convention: emit the linker/exports glue for the N-API .node addon.
extern crate napi_build;

fn main() {
    napi_build::setup();
}
