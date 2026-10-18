{
  description = "pico sdk shell";
  inputs.nixpkgs.url = "github:nixos/nixpkgs";

  outputs = {
    self,
    nixpkgs,
    ...
  }: let
    pkgs = nixpkgs.legacyPackages.x86_64-linux;
    local-pico-sdk = pkgs.pico-sdk.override {
      withSubmodules = true;
    };
  in {
    devShells.x86_64-linux.default = pkgs.mkShell {
      buildInputs = with pkgs; [
        gcc-arm-embedded
        local-pico-sdk
        picotool

        cmake
        python3 # build requirements for pico-sdk
        udisks # Interact with bootloader filesystem
        tio # terminal program to interface with serial
      ];

      packages = [
        (pkgs.python3.withPackages (python-pkgs:
          with python-pkgs; [
            pyserial
          ]))
      ];

      shellHook = ''
        export PICO_SDK_PATH=${local-pico-sdk}/lib/pico-sdk
      '';
    };
  };
}
