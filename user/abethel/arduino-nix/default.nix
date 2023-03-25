{ pkgs ? import <nixpkgs> { } }:
with pkgs;

# Alex's NixOS setup for arduino-cli.

let
  # arduino-cli core install <name>
  corePkgs = [
    "arduino:avr"
  ];

  # arduino-cli lib install <name>
  libPkgs = [
  ];

  # arduino-cli lib install --git-url <name>
  gitPkgs = [
    "https://github.com/Longan-Labs/Longan_CAN_MCP2515"
  ];

  # Hash of the output home dir.
  buildHash = "sha256-kQ9Yb1h7Fdt2IE88lgrVFiRBGwS6/ORzV1k7je/bH4M=";

  # --------

  # If we just point `HOME` at the build directory and tell Arduino
  # CLI to install the packages we need, it produces 3 types of
  # no-determinism: (1) package versions aren't pinned down and could
  # change over time; (2) ~/.arduino15/inventory.yaml gets 2 random
  # 256-bit numbers in it; and (3) ~/.arduino15/packages/arduino
  # /hardware/avr/1.8.6/installed.json has a list of installed
  # packages in a non-deterministic order.

  # Ignore (1) and update hashes every few days as necessary, because
  # fixing it would be too difficult; fix (2) by hardcoding `eeeee...`
  # as the random numbers; and fix (3) using a small Python program
  # that sorts the installed package list by name and version.

  inventory_yaml = writeText "inventory.yaml" ''
    installation:
      id: eeeeeeee-eeee-eeee-eeee-eeeeeeeeeeee
      secret: eeeeeeee-eeee-eeee-eeee-eeeeeeeeeeee
  '';

  arduino-home = stdenv.mkDerivation {
    name = "arduino-home";
    builder = writeScript "builder" ''
      #!${bash}/bin/bash
      set -e

      ${coreutils}/bin/mkdir -p $out
      export HOME=$out

      for corePkg in ${builtins.concatStringsSep " " corePkgs}; do
        ${arduino-cli}/bin/arduino-cli core install $corePkg
      done

      for pkg in ${builtins.concatStringsSep " " libPkgs}; do
        ${arduino-cli}/bin/arduino-cli lib install $pkg
      done

      export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
      for pkg in ${builtins.concatStringsSep " " gitPkgs}; do
        ${arduino-cli}/bin/arduino-cli lib install --git-url $pkg
      done

      ${python310}/bin/python ${./fixup_installed.py} $out/.arduino15/packages/arduino/hardware/avr/1.8.6/installed.json
      ${coreutils}/bin/cp ${inventory_yaml} $out/.arduino15/inventory.yaml
    '';

    SSL_CERT_FILE = "${cacert}/etc/ssl/certs/ca-bundle.crt";

    outputHash = buildHash;
    outputHashAlgo = "sha256";
    outputHashMode = "recursive";
  };

  arduino = writeScriptBin "arduino-cli" ''
    #!${bash}/bin/bash
    HOME=${arduino-home} ${arduino-cli}/bin/arduino-cli "$@"
  '';
in
arduino
