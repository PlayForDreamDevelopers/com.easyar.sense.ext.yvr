//================================================================================================================================
//
//  Copyright (c) 2015-2025 VisionStar Information Technology (Shanghai) Co., Ltd. All Rights Reserved.
//  EasyAR is the registered trademark or trademark of VisionStar Information Technology (Shanghai) Co., Ltd in China
//  and other countries for the augmented reality technology developed by VisionStar Information Technology (Shanghai) Co., Ltd.
//
//================================================================================================================================

#if EASYAR_ENABLE_SENSE
using UnityEditor;

namespace easyar
{
    using ExtensionFrameSource = YVRFrameSource;

    class MenuItems
    {
        const string name = "YVRFrameSource";

        [MenuItem(ExtensionMenuItemHelper.MenuPath + "Frame Source : " + name, priority = ExtensionMenuItemHelper.Priority)]
        static void AddFrameSource() => ExtensionMenuItemHelper.AddFrameSource<ExtensionFrameSource>();

        [MenuItem(ExtensionMenuItemHelper.MenuPath + "Frame Source : " + name + " (keep it only)", priority = ExtensionMenuItemHelper.Priority)]
        static void KeepFrameSourceOnly() => ExtensionMenuItemHelper.KeepFrameSourceOnly<ExtensionFrameSource>();

        [MenuItem(ExtensionMenuItemHelper.MenuPath + "Frame Source : " + name, true)]
        [MenuItem(ExtensionMenuItemHelper.MenuPath + "Frame Source : " + name + " (keep it only)", true)]
        static bool AddFrameSourceValidate() => ExtensionMenuItemHelper.AddFrameSourceValidate();
    }
}
#endif
