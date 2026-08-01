#include <CalibCore/CliArgs.h>
#include <cstdio>

namespace calib {

// A token is treated as a flag (and therefore stops value collection for the
// previous flag) when it starts with '-' and is more than a single '-'.
static bool isFlag(const char* tok) {
    return tok[0] == '-' && tok[1] != '\0';
}

CliArgs parseArgs(int argc, char* argv[]) {
    CliArgs a;
    int i = 1;
    while (i < argc) {
        const char* tok = argv[i];

        if (std::string(tok) == "-h" || std::string(tok) == "--help") {
            a.help = true;
            ++i;
            continue;
        }

        if (tok[0] == '-' && tok[1] == '-' && tok[2] != '\0') {
            std::string key = tok + 2;   // strip leading "--"
            ++i;
            // Collect every consecutive non-flag token as a value for this key.
            bool any = false;
            while (i < argc && !isFlag(argv[i])) {
                a.opts.emplace(key, argv[i]);
                ++i;
                any = true;
            }
            if (!any) a.opts.emplace(key, std::string{});  // valueless flag
        } else if (isFlag(tok)) {
            a.valid = false;
            a.error = std::string("unknown option: ") + tok;
            ++i;
        } else {
            a.positional.emplace_back(tok);
            ++i;
        }
    }
    return a;
}

void printUsage(const char* appName, const char* desc,
                const std::vector<std::string>& options, bool toStderr) {
    std::FILE* f = toStderr ? stderr : stdout;
    std::fprintf(f, "%s — %s\n\n", appName, desc);
    std::fprintf(f, "Usage: %s [options] [files...]\n\nOptions:\n", appName);
    for (const auto& o : options)
        std::fprintf(f, "%s\n", o.c_str());
    std::fprintf(f, "  -h, --help                   show this help and exit\n");
}

}  // namespace calib