#include <RaylibWidgets/PlatformKeys.h>
#include <raylib.h>

namespace raylib_widgets
{

    namespace
    {
#ifdef __APPLE__
        constexpr int kModifierLeft = KEY_LEFT_SUPER;
        constexpr int kModifierRight = KEY_RIGHT_SUPER;
#else
        constexpr int kModifierLeft = KEY_LEFT_CONTROL;
        constexpr int kModifierRight = KEY_RIGHT_CONTROL;
#endif
    } // namespace

    bool isShortcutModifierDown()
    {
        return IsKeyDown(kModifierLeft) || IsKeyDown(kModifierRight);
    }

    bool isShortcutModifierPressed()
    {
        return IsKeyPressed(kModifierLeft) || IsKeyPressed(kModifierRight);
    }

} // namespace raylib_widgets
