{ pkgs ? import <nixpkgs> {} }:
with pkgs;

mkShell {
  nativeBuildInputs = [
    (import ./default.nix)
  ];
}
