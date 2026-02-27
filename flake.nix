{
  description = "WPILib 2026 Java/Kotlin Development Shell";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { inherit system; };
        jdk = pkgs.temurin-bin-17;
      in
      {
        devShells.default = pkgs.mkShell {
          name = "wpilib-2026";

          buildInputs = with pkgs; [
            # Java / Kotlin
            jdk
            kotlin
            gradle

            # Native toolchain (needed for JNI / HAL builds)
            gcc
            cmake
            gnumake
          ];

          shellHook = ''
            export JAVA_HOME="${jdk.home}"
            export PATH="${jdk}/bin:$PATH"
            export GRADLE_USER_HOME="''${GRADLE_USER_HOME:-$HOME/.gradle}"

            # Point Gradle toolchain resolution at the Nix-provided JDK
            # and disable auto-provisioning (downloaded JDKs don't work on NixOS)
            export GRADLE_OPTS="-Dorg.gradle.java.installations.paths=${jdk.home} -Dorg.gradle.java.installations.auto-detect=false -Dorg.gradle.java.installations.auto-download=false"

            # WPILib JNI native libs need libstdc++ at runtime
            export LD_LIBRARY_PATH="${pkgs.stdenv.cc.cc.lib}/lib''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
          '';
        };
      }
    );
}
