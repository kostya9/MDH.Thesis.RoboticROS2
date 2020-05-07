package com.kostya9.rebeca_ros2_model_transformer.transformers;

import com.kostya9.rebeca_ros2_model_transformer.cpp.CPP_KEYWORD;
import com.kostya9.rebeca_ros2_model_transformer.cpp.CppCodeEmitter;
import jdk.jshell.spi.ExecutionControl;
import org.rebecalang.compiler.modelcompiler.corerebeca.objectmodel.FieldDeclaration;
import org.rebecalang.compiler.modelcompiler.corerebeca.objectmodel.OrdinaryPrimitiveType;
import org.rebecalang.compiler.modelcompiler.corerebeca.objectmodel.Type;
import org.rebecalang.compiler.utils.TypesUtilities;

import java.io.FileWriter;
import java.io.IOException;
import java.util.function.Function;

public class CppTransformer {
    private final CppCodeEmitter emitter;

    public CppTransformer(String file) throws IOException {
        emitter = new CppCodeEmitter(new FileWriter(file));
    }

    public void handleFieldDeclaration(FieldDeclaration field) throws IOException {
        var type = field.getType();
        if(type instanceof OrdinaryPrimitiveType) {
            var primitive = (OrdinaryPrimitiveType) type;

            Function<Type, Boolean> ofType = (t) -> t == primitive;

            if(type == TypesUtilities.BOOLEAN_TYPE) {
                emitter.keyword(CPP_KEYWORD.BOOL);
            }
            else if (type == TypesUtilities.BYTE_TYPE) {
                emitter.keyword(CPP_KEYWORD.BYTE);
            }
            else if(type == TypesUtilities.CHAR_TYPE) {
                emitter.keyword(CPP_KEYWORD.CHAR);
            }
            else if(type == TypesUtilities.DOUBLE_TYPE) {
                emitter.keyword(CPP_KEYWORD.DOUBLE);
            }
            else if(type == TypesUtilities.FLOAT_TYPE) {
                emitter.keyword(CPP_KEYWORD.FLOAT);
            }
            else if(type == TypesUtilities.INT_TYPE) {
                emitter.keyword(CPP_KEYWORD.INT);
            }
            else if(type == TypesUtilities.SHORT_TYPE) {
                emitter.keyword(CPP_KEYWORD.SHORT);
            }
            else if(type == TypesUtilities.STRING_TYPE) {
                emitter.type(KnownCppTypes.String.type, KnownCppTypes.String.namespaceElements);
            }
            else {
                throw new UnsupportedOperationException("The model transformer does not support " + primitive.getName() + " type");
            }


            for (var declaration : field.getVariableDeclarators()) {
                emitter.emit(declaration.getVariableName());

                if(declaration.getVariableInitializer() != null) {
                    emitter.signEqual();
                    // TODO: SUPPORT INITIALIZER
                    throw new ExecutionControl.NotImplementedException("Needto add initializer support");
                }
                emitter.comma();
            }

        }
    }
}
