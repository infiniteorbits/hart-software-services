set dotenv-load


export BINDIR    := "build"
COMPILE_DB     := "compile_commands.json"

build:
    #!/usr/bin/env bash
    source .env
    conan install . --profile io-sw-pld-boot --output-folder={{BINDIR}}
    # TODO Forward board value from profile
    bear --append -- make

clean:
    rm -fR {{BINDIR}} {{COMPILE_DB}}

