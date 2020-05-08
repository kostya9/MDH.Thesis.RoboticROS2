package com.kostya9.rebeca_ros2_model_transformer.transformers;

import com.kostya9.rebeca_ros2_model_transformer.cpp.CPP_KEYWORD;
import com.kostya9.rebeca_ros2_model_transformer.cpp.CppCodeEmitter;
import org.rebecalang.compiler.modelcompiler.corerebeca.objectmodel.*;
import org.rebecalang.compiler.utils.TypesUtilities;

import java.io.IOException;
import java.util.List;
import java.util.Random;

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

                if (i != variableDeclarators.size() - 1) {
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

                if (i != values.size() - 1) {
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
            emitter.startParenthesis();
            handleExpression(plusSub.getValue());
            emitter.endParenthesis();
            emitter.emit(plusSub.getOperator());
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

    public void handleStatement(Statement statement) throws IOException {
        if(statement instanceof BlockStatement) {
            var block = (BlockStatement) statement;
            emitter.startBracket();
            for (var nestedStatement : block.getStatements()) {
                handleStatement(nestedStatement);
            }
            emitter.endBracket();
            emitter.endLine();
        }
        else if(statement instanceof BreakStatement) {
            emitter.keyword(CPP_KEYWORD.BREAK);
            emitter.endStatement();
            emitter.endLine();
        }
        else if(statement instanceof ConditionalStatement) {
            var conditional = (ConditionalStatement) statement;

            emitter.keyword(CPP_KEYWORD.IF);
            emitter.startBracket();
            handleExpression(conditional.getCondition());
            emitter.endBracket();
            emitter.endLine();
        }
        else if(statement instanceof ContinueStatement) {
            emitter.keyword(CPP_KEYWORD.CONTINUE);
            emitter.endStatement();
            emitter.endLine();
        }
        else if(statement instanceof Expression) {
            var expression = (Expression) statement;
            handleExpression(expression);
            emitter.endStatement();
            emitter.endLine();
        }
        else if(statement instanceof FieldDeclaration) {
            var field = (FieldDeclaration) statement;
            handleFieldDeclaration(field);
            emitter.endStatement();
            emitter.endLine();
        }
        else if(statement instanceof ForStatement) {
            var forStatement = (ForStatement) statement;

            emitter.keyword(CPP_KEYWORD.FOR);
            emitter.startParenthesis();
            handleExpression(forStatement.getCondition());
            emitter.endStatement();
            var forInitializer = forStatement.getForInitializer();
            handleFieldDeclaration(forInitializer.getFieldDeclaration());
            handleCommaExpressionList(forInitializer.getExpressions());
            emitter.endStatement();
            handleCommaExpressionList(forStatement.getForIncrement());
            emitter.endStatement();
            emitter.endParenthesis();
            emitter.endLine();
        }
        else if(statement instanceof ReturnStatement) {
            var returnStatement = (ReturnStatement) statement;

            emitter.keyword(CPP_KEYWORD.RETURN);
            handleExpression(returnStatement.getExpression());
            emitter.endLine();
        }
        else if(statement instanceof WhileStatement) {
            var whileStatement = (WhileStatement) statement;

            emitter.keyword(CPP_KEYWORD.WHILE);
            emitter.startParenthesis();
            handleExpression(whileStatement.getCondition());
            emitter.endParenthesis();
            emitter.endLine();
        }
        else if(statement instanceof SwitchStatement) {
            var switchStatement = (SwitchStatement) statement;

            emitter.keyword(CPP_KEYWORD.SWITCH);
            emitter.startParenthesis();
            handleExpression(switchStatement.getExpression());
            emitter.endParenthesis();
            emitter.endLine();
            emitter.startBracket();
            for (var group : switchStatement.getSwitchStatementGroups()) {
                emitter.keyword(CPP_KEYWORD.CASE);
                handleExpression(group.getExpression());
                emitter.emit(':');
                emitter.endLine();
                emitter.startBracket();
                for (var innerStatement : group.getStatements()) {
                    handleStatement(innerStatement);
                }
                emitter.endBracket();
                emitter.endLine();
            }
            emitter.endBracket();
        }
    }

    private void handleCommaExpressionList(List<Expression> expressions) throws IOException {
        for (int i = 0; i < expressions.size(); i++) {
            var expression = expressions.get(i);
            handleExpression(expression);

            if(i != expressions.size() - 1) {
                emitter.comma();
            }
        }
    }
}
