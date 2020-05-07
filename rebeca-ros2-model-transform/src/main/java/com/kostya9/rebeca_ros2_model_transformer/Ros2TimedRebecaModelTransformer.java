package com.kostya9.rebeca_ros2_model_transformer;

import com.kostya9.rebeca_ros2_model_transformer.transformers.EnvironmentVariables;
import org.rebecalang.compiler.modelcompiler.SymbolTable;
import org.rebecalang.compiler.modelcompiler.corerebeca.objectmodel.RebecaCode;
import org.rebecalang.compiler.modelcompiler.corerebeca.objectmodel.RebecaModel;

public class Ros2TimedRebecaModelTransformer {
    private final RebecaCode rebecaCode;
    private final SymbolTable symbolTable;

    public Ros2TimedRebecaModelTransformer(RebecaModel model, SymbolTable symbolTable) {
        this.rebecaCode = model.getRebecaCode();
        this.symbolTable = symbolTable;
    }

    public void transformModel() {
        var environmentVariables = new EnvironmentVariables(rebecaCode.getEnvironmentVariables());
    }
}
