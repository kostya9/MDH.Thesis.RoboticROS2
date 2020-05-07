package com.kostya9.rebeca_ros2_model_transformer.transformers;

import org.rebecalang.compiler.modelcompiler.corerebeca.objectmodel.FieldDeclaration;
import org.rebecalang.compiler.modelcompiler.corerebeca.objectmodel.Type;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class EnvironmentVariables {
    private List<FieldDeclaration> environmentVariables;

    public EnvironmentVariables(List<FieldDeclaration> environmentVariables) {
    }

    public void cppToFile(File file) throws IOException {
        /*var writer = new FileWriter(file);

        for (var env : environmentVariables) {
            env.getType() == Type.
        }
        writer.write();*/
    }
}
