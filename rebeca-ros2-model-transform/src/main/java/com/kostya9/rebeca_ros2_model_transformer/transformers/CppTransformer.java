package com.kostya9.rebeca_ros2_model_transformer.transformers;

import com.kostya9.rebeca_ros2_model_transformer.cpp.CPP_KEYWORD;
import com.kostya9.rebeca_ros2_model_transformer.cpp.CppCodeEmitter;
import org.rebecalang.compiler.modelcompiler.corerebeca.objectmodel.*;
import org.rebecalang.compiler.modelcompiler.probabilisticrebeca.objectmodel.ProbabilisticExpression;
import org.rebecalang.compiler.utils.TypesUtilities;

import java.io.IOException;
import java.util.Random;
import java.util.function.Function;

public class CppTransformer {
    private final CppCodeEmitter emitter;
    private final Random random;

    public CppTransformer(CppCodeEmitter emitter) throws IOException {
        this.emitter = emitter;
        this.random = new Random();
    }

    public void handleFieldDeclaration(FieldDeclaration field) throws IOException {
        var type = field.getType();
        if(type instanceof OrdinaryPrimitiveType) {
            var primitive = (OrdinaryPrimitiveType) type;

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

                if (i != values.size()) {
                    emitter.comma();
                }
            }
        }
        else {
            throw new UnsupportedOperationException("Variable initializer of type " + initializer.getType().toString() + " is not supported");
        }
    }

    public void handleExpression(Expression expression) throws IOException {
        if (expression instanceof BinaryExpression) {
            var binary = (BinaryExpression) expression;

            handleExpression(binary.getLeft());
            emitter.emit(binary.getOperator());
            handleExpression(binary.getRight());
        }
        else if (expression instanceof UnaryExpression) {
            var unary = (UnaryExpression) expression;
            emitter.emit(unary.getOperator());
        }
        else if(expression instanceof TernaryExpression) {
            var ternary = (TernaryExpression) expression;

            handleExpression(ternary.getCondition());
            emitter.emit('?');
            handleExpression(ternary.getLeft());
            emitter.emit(':');
            handleExpression(ternary.getRight());
        }
        else if(expression instanceof NonDetExpression) {
            var nonDet = (NonDetExpression) expression;
            var choices = nonDet.getChoices();

            var choiceIdx = random.nextInt(choices.size());
            var choice = nonDet.getChoices().get(choiceIdx);
            handleExpression(choice);
        }
        else if(expression instanceof PlusSubExpression) {
            var plusSub = (PlusSubExpression) expression;
            emitter.emit('(');
            handleExpression(plusSub.getValue());
            emitter.emit(')');
        }
        else if(expression instanceof Literal) {
            var literal = (Literal) expression;

            emitter.emit(literal.getLiteralValue());
        }/*
        else if(expression instanceof PrimaryExpression) {
            // TODO: add primary handling
        }*/
        else {
            throw new IOException("Unsupported expression " + expression.getType());
        }
    }
}
