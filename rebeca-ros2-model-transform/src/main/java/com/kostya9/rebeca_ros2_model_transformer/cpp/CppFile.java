package com.kostya9.rebeca_ros2_model_transformer.cpp;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class CppFile {
    public static CppCodeEmitter Create(String path) throws IOException {
        var file = new File(path);
        if(file.exists()) {
            file.delete();
        }

        var writer = new FileWriter(path);
        return new CppCodeEmitter(writer);
    }
}
