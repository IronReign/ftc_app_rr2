package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayDeque;


public class FixedCache<T> extends ArrayDeque<T> {
    private int size;
    public FixedCache(int size) {
        super(size);
        this.size = size;
    }

    @Override
    public boolean add(T object) {
        while (size() >= size)
            remove();
        return super.add(object);
    }

}
