# Agent Commands

Run all commands from the repository root.

## Build

```bash
cargo +nightly build --release
```

## Format

Apply formatting:

```bash
cargo +nightly fmt --all
```

Check formatting (CI style):

```bash
cargo +nightly fmt --all -- --check
```

## Lint

```bash
cargo +nightly clippy --all-features --workspace -- -D warnings
```

## One-time setup (if needed)

```bash
rustup toolchain install nightly --component rust-src rustfmt clippy
rustup target add riscv32imc-unknown-none-elf --toolchain nightly
```
