# System32 emulator flake.
#   all systems:  packages.emulator (default: the native front ends, built and unit-tested in the sandbox),
#                 devShells.default (everything `make check` needs: C compiler, Python + uv, Node, Emscripten,
#                 cppcheck, Playwright's Chromium on Linux), checks.verify
#
# flake.lock fixes the nixpkgs revisions, so the native compiler, Python, uv, Node and Emscripten are the same builds
# on every machine, and an emulator built anywhere gets the same build stamp from the same sources
# (emu/common/build_id.sh). nix/tool-versions.txt names the versions;
# evaluation stops with an error if nixpkgs and that file disagree. Python packages are pinned by uv.lock
# (pyproject.toml) and run through scripts/pyrun.
{
  description = "System32 emulator: the EFII System32 hardware (Cockpit Controller + two ECUs), native and WebAssembly";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    # Emscripten only. nixos-24.11's emscripten 3.1.64 comes with binaryen 118, whose wasm-opt rejects the
    # --no-stack-ir option emcc passes at -O3; nixos-25.05's 4.0.10 + binaryen 123 is a matched pair (tested).
    nixpkgs-emscripten.url = "github:NixOS/nixpkgs/nixos-25.05";
  };

  outputs = { self, nixpkgs, nixpkgs-emscripten }:
    let
      lib = nixpkgs.lib;
      supportedSystems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      forAllSystems = lib.genAttrs supportedSystems;
      pkgsFor = system: import nixpkgs { inherit system; };
      emscriptenFor = system: (import nixpkgs-emscripten { inherit system; }).emscripten;

      # nix/tool-versions.txt: "<tool> <version>" lines, '#' comments.
      pinnedVersions = builtins.listToAttrs (map
        (line: let w = lib.splitString " " line; in { name = builtins.elemAt w 0; value = builtins.elemAt w 1; })
        (builtins.filter (l: l != "" && !(lib.hasPrefix "#" l))
          (lib.splitString "\n" (builtins.readFile ./nix/tool-versions.txt))));
      pinned = name: pkg:
        if pkg.version == pinnedVersions.${name} then pkg
        else throw "nix/tool-versions.txt pins ${name} ${pinnedVersions.${name}} but nixpkgs has ${pkg.version}";

      toolsFor = pkgs: {
        emscripten = pinned "emscripten" (emscriptenFor pkgs.stdenv.hostPlatform.system);
        python = pinned "python" pkgs.python312;
        uv = pinned "uv" pkgs.uv;
        node = pinned "node" pkgs.nodejs_20;
        cppcheck = pinned "cppcheck" pkgs.cppcheck;
      };
    in
    {
      packages = forAllSystems (system:
        let
          pkgs = pkgsFor system;
          tools = toolsFor pkgs;
        in
        {
          # The native emulator, built and unit-tested in the Nix sandbox (no network, so nothing from uv.lock and no
          # firmware: the C unit tests and the synthetic-image tests run; firmware-dependent checks SKIP).
          emulator = pkgs.stdenv.mkDerivation {
            pname = "system32-emulator";
            version = "0.1.0";
            src = ./.;
            nativeBuildInputs = [ tools.python pkgs.perl pkgs.rsync ];
            buildInputs = [ pkgs.zlib ];
            postPatch = ''
              patchShebangs scripts emu
            '';
            buildPhase = ''
              make emu
            '';
            doCheck = true;
            # The C unit tests and the front end's shell tests. The tooling tests (make test-scripts) need a git
            # checkout (one lists the tracked files), which the sandbox's source copy is not; `make check` runs them.
            checkPhase = ''
              make test-c
            '';
            installPhase = ''
              mkdir -p $out/bin
              cp emu/pair/build/s32pair_shim $out/bin/
            '';
          };
          default = self.packages.${system}.emulator;
        });

      checks = forAllSystems (system: { verify = self.packages.${system}.emulator; });

      devShells = forAllSystems (system:
        let
          pkgs = pkgsFor system;
          tools = toolsFor pkgs;
          # Playwright's Chromium from nixpkgs on Linux; elsewhere `scripts/pyrun -m playwright install chromium`.
          chromium = if pkgs.stdenv.isLinux then (pinned "playwright-chromium" pkgs.playwright-driver).browsers-chromium else null;
        in
        {
          default = pkgs.mkShell {
            packages = [
              tools.python
              tools.uv
              tools.node
              tools.emscripten
              tools.cppcheck
              pkgs.clang
              pkgs.gnumake
              pkgs.git
              pkgs.perl
              pkgs.rsync
              pkgs.zlib
            ];
            PINNED = "1";
            UV_PYTHON = "${tools.python}/bin/python3";
            UV_PYTHON_DOWNLOADS = "never";
            PLAYWRIGHT_NODEJS_PATH = "${tools.node}/bin/node";
            S32_PYRUN_LIBRARY_PATH = lib.optionalString pkgs.stdenv.isLinux
              (lib.makeLibraryPath [ pkgs.stdenv.cc.cc.lib pkgs.zlib ]);
            shellHook = ''
              export EM_CACHE="''${XDG_CACHE_HOME:-$HOME/.cache}/system32-emulator/emscripten-${tools.emscripten.version}"
              if [ ! -d "$EM_CACHE" ]; then
                mkdir -p "$EM_CACHE"
                cp -R --no-preserve=mode ${tools.emscripten}/share/emscripten/cache/. "$EM_CACHE"/ 2>/dev/null || true
              fi
              ${lib.optionalString pkgs.stdenv.isLinux ''
                export PLAYWRIGHT_BROWSERS_PATH="${chromium}"
                export PLAYWRIGHT_SKIP_VALIDATE_HOST_REQUIREMENTS=true
              ''}
              if [ -z "''${S32_QUIET:-}" ]; then
                echo "System32 emulator pinned environment (nix/tool-versions.txt, uv.lock):"
                echo "  make check      native + WebAssembly builds, unit tests, conformance (native = WebAssembly)"
                echo "  make firmware   fetch and verify the default firmware (S32_V86_SRC=<dir> for a local copy)"
              fi
            '';
          };
        });
    };
}
