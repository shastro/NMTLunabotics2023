{ pkgs ? import <nixpkgs> { } }:
with pkgs.python310Packages;

let
  python-can = buildPythonPackage rec {
    pname = "python-can";
    version = "4.1.0";

    src = fetchPypi {
      inherit pname version;
      sha256 = "sha256-PytrDcX0WVkdFx7gwBNtznms7cJ0DOaVAkqjRE6RG7k=";
    };
    propagatedBuildInputs = [
      msgpack
      packaging
      typing-extensions
      wrapt
    ];
    doCheck = false;
  };
  textparser = buildPythonPackage rec {
    pname = "textparser";
    version = "0.24.0";

    src = fetchPypi {
      inherit pname version;
      sha256 = "sha256-VvcI51qp0AKtt22CO6bvFm1+zsHj5MpMHKED+BdWgzU=";
    };
    propagatedBuildInputs = [
    ];
    doCheck = false;
  };
in
buildPythonPackage rec {
  pname = "cantools";
  version = "38.0.2";

  src = fetchPypi {
    inherit pname version;
    sha256 = "sha256-k7/m9L1lLzaXY+qRYrAnpi9CSoQA8kI9QRN5GM5oxo4=";
  };

  propagatedBuildInputs = [
    argparse-addons
    bitstruct
    crccheck
    diskcache
    python-can
    setuptools
    textparser
  ];

  doCheck = false;
}
