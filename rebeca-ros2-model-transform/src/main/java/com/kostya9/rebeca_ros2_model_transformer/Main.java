package com.kostya9.rebeca_ros2_model_transformer;

import org.rebecalang.compiler.modelcompiler.RebecaCompiler;
import org.rebecalang.compiler.utils.CompilerFeature;

import java.io.File;
import java.util.HashSet;
import java.util.Set;

public class Main {

    public static void main(String[] args) {
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
    }
}
