package org.firstinspires.ftc.teamcode.subsystems;

/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

import org.firstinspires.ftc.robotcore.external.tfod.TfodBase;

/**
 * A class that provides simplified access to TensorFlow Object Detection for the current game.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
public class TfodTSE extends TfodBase {
    // TODO(lizlooney): This file should be updated when the FTC game changes.
    public static final String TFOD_MODEL_ASSET = "model_20220117_133234.tflite";

    public static final String[] LABELS = {
            "TSE",
    };

    private static final boolean IS_MODEL_TENSOR_FLOW_2 = true;

    public TfodTSE() {
        //super(TFOD_MODEL_ASSET, LABELS, IS_MODEL_TENSOR_FLOW_2);
        super(null, LABELS, IS_MODEL_TENSOR_FLOW_2);
        // P-TSE setModelFromFile("/storage/emulated/0/FIRST/tflitemodels/purpleTSE_20220213_210831.tflite", LABELS);
        setModelFromFile( "/storage/emulated/0/FIRST/tflitemodels/model_20220117_133234.tflite", LABELS);
    }
}
