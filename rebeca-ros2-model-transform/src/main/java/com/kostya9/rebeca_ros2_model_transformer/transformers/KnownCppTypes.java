package com.kostya9.rebeca_ros2_model_transformer.transformers;

import java.util.List;

public class KnownCppTypes {
    public static final KnownCppTypes String = new KnownCppTypes("string", "std");

    public final String type;
    public final Iterable<String> namespaceElements;

    public KnownCppTypes(String type, String... namespaceElements) {
        this.type = type;
        this.namespaceElements = List.of(namespaceElements);
    }
}
