package com.kostya9.rebeca_ros2_model_transformer.cpp;

import java.io.IOException;
import java.io.Writer;

public class CppCodeEmitter {
    private static final int WHITESPACE_PER_NESTING = 4;
    private static final char WHITESPACE_INDENT = ' ';

    private final Writer writer;

    private int nestingLevel;

    public CppCodeEmitter(Writer writer) {
        this.writer = writer;
    }

    public void startParenthesis() throws IOException {
        writer.append('(');
    }

    public void comma() throws IOException {
        writer.append(',');
        writer.append(' ');
    }

    public void endParenthesis() throws IOException {
        writer.append(')');
    }

    public void startBracket() throws IOException {
        writer.append('{');
        nestingLevel++;
    }

    public void label(String name) throws IOException {
        writer.write(name);
        writer.write(':');
    }

    public void genericType(String type, String inner) throws IOException {
        writer.write(type);
        writer.append('<');
        writer.write(inner);
        writer.append('>');
        writer.append(' ');
    }

    public void endBracket() throws IOException {
        if(nestingLevel > 0) {
            nestingLevel--;
        }

        writer.append('}');
    }

    private static final String INCLUDE_KW = "#include";
    public void include(String header) throws IOException {
        writer.write(INCLUDE_KW);
        writer.append(' ');
        writer.append('<');
        writer.write(header);
        writer.append('>');
        emitEndStatement();
    }

    public void emitEndStatement() throws IOException {
        writer.append(';');
    }

    public void endLine() throws IOException {
        writer.write(System.lineSeparator());
        emitNesting();
    }

    public void keyword(CPP_KEYWORD kw) throws IOException {
        writer.write(kw.toString().toLowerCase());
        writer.append(' ');
    }

    public void emit(String value) throws IOException {
        writer.write(value);
    }

    public void emit(char c) throws IOException {
        writer.append(c);
    }

    private void emitNesting() throws IOException {
        int totalSpaces = nestingLevel * WHITESPACE_PER_NESTING;
        for (int i = 0; i < totalSpaces; i++) {
            writer.append(WHITESPACE_INDENT);
        }
    }

    private static final String NAMESPACE_SEPARATOR = "::";
    public void type(String type, Iterable<String> namespaceElements) throws IOException {
        for (var namespace : namespaceElements) {
            writer.write(namespace);
            writer.write(NAMESPACE_SEPARATOR);
        }

        writer.write(type);
    }

    public void signEqual() throws IOException {
        writer.append('=');
    }

    public void flush() throws IOException {
        writer.flush();
    }

    public void close() throws IOException {
        writer.close();
    }
}

