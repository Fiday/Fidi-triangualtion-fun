package org.delaunay.model;


public class Pair<K,V> {
    K Key;
    V Value;


    public Pair(K key, V value) {
        Key = key;
        Value = value;
    }

    public Pair() {
    }

    public K getKey() {
        return Key;
    }

    public void setKey(K key) {
        Key = key;
    }

    public V getValue() {
        return Value;
    }

    public void setValue(V value) {
        Value = value;
    }


}
