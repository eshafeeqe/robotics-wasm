use wasm_bindgen::prelude::*;

// This function acts like "print" but sends it to the browser's console
#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

// This is the function we will call from JavaScript
#[wasm_bindgen]
pub fn hello_robotics(name: &str) -> String {
    // 1. Print to the browser console using the function we defined above
    log(&format!("Rust received: {}", name));
    
    // 2. Return a string back to JavaScript
    format!("Hello, {}! This message came from Rust.", name)
}