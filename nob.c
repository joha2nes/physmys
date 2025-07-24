#ifndef NOB_IMPLEMENTATION
#define NOB_IMPLEMENTATION

#include "nob.h"

void libs(Nob_Cmd* cmd)
{
    nob_cmd_append(cmd,
        "-Ilib/raylib/", 
        "-framework", "CoreVideo", 
        "-framework", "IOKit", 
        "-framework", "Cocoa", 
        "-framework", "GLUT", 
        "-framework", "OpenGL", 
        "lib/raylib/libraylib.a");
}

void cc(Nob_Cmd* cmd)
{
    nob_cmd_append(cmd, "clang");
    nob_cmd_append(cmd, "-Wall", "-Wextra");
}

bool buildMain(Nob_Cmd* cmd)
{
    cmd->count = 0;
    cc(cmd);
    nob_cmd_append(cmd, "src/main.c", "src/debug_draw.c");
    nob_cmd_append(cmd, "-o", "./main");
    libs(cmd);
    return nob_cmd_run_sync(*cmd);
}

int main(int argc, char** argv)
{
    NOB_GO_REBUILD_URSELF(argc, argv);

    Nob_Cmd cmd = {0};
    if (!buildMain(&cmd)) return 1;

    cmd.count = 0;
    nob_cmd_append(&cmd, "./main");
    nob_cmd_run_sync(cmd);

    return 0;
}

#endif // NOB_IMPLEMENTATION