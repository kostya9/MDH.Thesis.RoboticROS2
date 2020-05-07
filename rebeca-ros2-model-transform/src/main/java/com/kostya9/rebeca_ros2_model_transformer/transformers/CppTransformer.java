package com.kostya9.rebeca_ros2_model_transformer.transformers;

import com.kostya9.rebeca_ros2_model_transformer.cpp.CPP_KEYWORD;
import com.kostya9.rebeca_ros2_model_transformer.cpp.CppCodeEmitter;
import org.rebecalang.compiler.modelcompiler.corerebeca.objectmodel.*;
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


            var variableDeclarators = field.getVariableDeclarators();
            for (int i = 0; i < variableDeclarators.size(); i++) {
                var declaration = variableDeclarators.get(i);
                emitter.emit(declaration.getVariableName());

                var initializer = declaration.getVariableInitializer();
                if (initializer != null) {
                    emitter.signEqual();
                    handleVariableInitializer(initializer);
                }

                if (i != variableDeclarators.size()) {
                    emitter.comma();
                }
            }
        }
    }

    private void handleVariableInitializer(VariableInitializer initializer) throws IOException {
        if (initializer instanceof OrdinaryVariableInitializer) {
            var ordinaryInitializer = (OrdinaryVariableInitializer) initializer;
            handleExpression(ordinaryInitializer.getValue());
        }
        else if (initializer instanceof ArrayVariableInitializer) {
            var arrayInitializer = (ArrayVariableInitializer) initializer;

            var values = arrayInitializer.getValues();
            for (int i = 0; i < values.size(); i++) {
                var nestedInitializer = values.get(i);

                handleVariableInitializer(nestedInitializer);

                if(i != values.size()) {
                    emitter.comma();
                }
            }
        }
        else {
            throw new UnsupportedOperationException("Variable initializer of type " + initializer.getType().toString() + " is not supported");
        }
    }

    public void handleExpression(Expression expression) throws IOException {
        // TODO: Add support for expression parsing
        throw new IOException("Not implemented");
    }
}
