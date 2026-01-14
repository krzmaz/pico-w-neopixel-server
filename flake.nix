{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    flake-utils.url = "github:numtide/flake-utils";
    naersk.url = "github:nix-community/naersk";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs = {
        nixpkgs.follows = "nixpkgs";
      };
    };
  };
  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      rust-overlay,
      naersk,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };
        target = "thumbv6m-none-eabi";
        rust = pkgs.rust-bin.stable.latest.default.override {
          targets = [ target ];
        };
        naersk' = pkgs.callPackage naersk {
          rustc = rust;
          cargo = rust;
        };
        pico-w-neopixel-server-elf = naersk'.buildPackage {
          pname = "pico-w-neopixel-server";
          src = pkgs.runCommand "src-with-firmware-blobs" { } ''
            mkdir $out
            cp -r ${./.}/* $out/
          '';
          nativeBuildInputs = [
            pkgs.git
          ];
          CARGO_BUILD_TARGET = target;
          # strangely, this was stripping nearly the entire binary
          dontStrip = true;
        };
        pico-w-neopixel-server-uf2 =
          pkgs.runCommand "elf-to-uf2"
            {
              nativeBuildInputs = [ pkgs.elf2uf2-rs ];
            }
            ''
              mkdir -p $out/bin
              elf2uf2-rs ${pico-w-neopixel-server-elf}/bin/tcp $out/bin/pico-w-neopixel-server-tcp.uf2
              elf2uf2-rs ${pico-w-neopixel-server-elf}/bin/udp $out/bin/pico-w-neopixel-server-udp.uf2
            '';
      in
      with pkgs;
      rec {
        packages = {
          inherit pico-w-neopixel-server-elf pico-w-neopixel-server-uf2;
          default = pico-w-neopixel-server-uf2;
        };
        apps = rec {
          default = upload;
          upload = {
            type = "app";
            program = pkgs.lib.getExe (pkgs.writeShellScriptBin "upload" "cargo run --release");
          };
        };
        devShells.default = mkShell {
          nativeBuildInputs =
            self.packages.${system}.pico-w-neopixel-server-elf.nativeBuildInputs
            ++ self.packages.${system}.pico-w-neopixel-server-uf2.nativeBuildInputs
            ++ [
              picotool
              netcat
            ];
        };
      }
    );
}
