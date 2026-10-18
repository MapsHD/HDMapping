#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest.h>

#include <cstdio>
#include <cstring>

// `easy_gui_tests --echo-args <args...>` prints each argument as "[arg]" on its own line and exits: a child program
// that behaves the same on every platform, for the ChildProcess argument quoting test.
int main(int argc, char** argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--echo-args") == 0)
    {
        for (int i = 2; i < argc; i++)
            std::printf("[%s]\n", argv[i]);
        return 0;
    }
    doctest::Context context;
    context.applyCommandLine(argc, argv);
    return context.run();
}
