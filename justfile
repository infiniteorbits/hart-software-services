set dotenv-load


BUILD_ROOT    := "build"

COMPILE_DB     := "compile_commands.json"




build:
    #!/usr/bin/env bash
    source .env
    bear -- make

clean:
    rm -fR {{BUILD_ROOT}} {{COMPILE_DB}}

