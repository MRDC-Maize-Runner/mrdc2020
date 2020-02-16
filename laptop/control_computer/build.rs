fn main() {
    prost_build::compile_protos(&["src/protos/state.proto"],
                                &["src/"]).unwrap();
}