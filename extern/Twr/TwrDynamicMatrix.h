// This file is part of TowerEngine
// Copyright (C) 2013 Arnaud Emilien <arnaud.emilien@inria.fr>

#ifndef TWR_DYNAMIC_MATRIX_H
#define TWR_DYNAMIC_MATRIX_H


template <class T>
class TwrDynamicMatrix{

protected :
    T** 	 _pointers;
    T*           _data;
    unsigned int _nbLines;
    unsigned int _nbColumns;

public :
    TwrDynamicMatrix(unsigned int nbColumns, unsigned int nbLines);
    TwrDynamicMatrix(const TwrDynamicMatrix<T>& matrix);
    ~TwrDynamicMatrix();

    unsigned int getNbLines()   const { return _nbLines;   }
    unsigned int getNbColumns() const { return _nbColumns; }

    void setAt(unsigned int column, unsigned int line, const T& value);
    T&   getAt(unsigned int column, unsigned int line);
    
    void clear();
    
    T*   operator[](int line);
    T* getData() { return _data; }

}; // class TwrDynamicMatrix

#include "TwrDynamicMatrix.impl"


#endif // TWR_DYNAMIC_MATRIX_H
