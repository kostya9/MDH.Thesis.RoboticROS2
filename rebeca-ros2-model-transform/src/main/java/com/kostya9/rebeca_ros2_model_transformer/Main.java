package com.kostya9.rebeca_ros2_model_transformer;

import com.kostya9.rebeca_ros2_model_transformer.cpp.CppFile;
import com.kostya9.rebeca_ros2_model_transformer.transformers.CppTransformer;
import org.rebecalang.compiler.modelcompiler.RebecaCompiler;
import org.rebecalang.compiler.utils.CompilerFeature;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;

public class Main {

    public static void main(String[] args) throws IOException {
        var file = new File(
                Main.class.getResource("/testRebeca.rebeca").getFile()
        );
        var compiler = new RebecaCompiler();
        var features = new HashSet<CompilerFeature>();
        features.add(CompilerFeature.CORE_2_3);
        features.add(CompilerFeature.TIMED_REBECA);
        var compiled = compiler.compileRebecaFile(file, features);

        var model = compiled.getFirst();
        var symbols = compiled.getSecond();

        var rebeca = model.getRebecaCode();

        var tempDir = System.getProperty("java.io.tmpdir");

        var filePath = tempDir + "/code.cpp";

        var emitter = CppFile.Create(filePath);
        var transformer = new CppTransformer(emitter);
        transformer.handleFieldDeclaration(rebeca.getReactiveClassDeclaration().get(0).getStatevars().get(0));
        emitter.flush();
        emitter.close();
    }
}
