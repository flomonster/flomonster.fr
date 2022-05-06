{ pkgs ? import <nixpkgs> {} }:
let
make_packages = ps: [
        ps.jinja2
        ps.pypandoc
        ps.pyyaml
    ];
in
pkgs.mkShell {
  buildInputs = with pkgs; [
    (pkgs.python3.withPackages make_packages)
    pandoc
  ];
}
