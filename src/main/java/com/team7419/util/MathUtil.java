// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team7419.util;

/** Add your docs here. */
public class MathUtil {
    public static double mappingClamp(double value, double fromMin, double fromMax, double toMin, double toMax) {
        return(toMin + (value - fromMin) * (toMax - toMin) / (fromMax - fromMin));
    }
}
