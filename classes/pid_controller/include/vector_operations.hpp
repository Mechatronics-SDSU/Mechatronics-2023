#ifndef VECTOR_OPERATIONS_H
#define VECTOR_OPERATIONS_H

#define _USE_MATH_DEFINES
#define PI 3.14159265
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// Print each element of vector
template <typename T>
void printVector(vector<T>& vector) 
{
    for (T element : vector)
    {
        cout << element << " ";
    }
    cout << endl;
}

// Decrement two vectors
template <typename T>
vector<T>& operator-=(vector<T> &lhs, const vector<T> &rhs) 
{
    if (lhs.size() != rhs.size())
        throw length_error("vectors must be same size to add");
    for (vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] -= rhs[i];
    return lhs;
}

// Subtract two vectors
template <typename T>
vector<T> operator- (vector<T> lhs, const vector<T> &rhs) 
{
    if (lhs.size() != rhs.size())
        throw length_error("vectors must be same size to add");
    return lhs -= rhs;
}

// Increment two vectors
template <typename T>
vector<T>& operator+=(vector<T> &lhs, const vector<T> &rhs) 
{
    if (lhs.size() != rhs.size())
        throw length_error("vectors must be same size to add");
    for (vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] += rhs[i];
    return lhs;
}

// Add two vectors
template <typename T>
vector<T> operator+ (vector<T> lhs, const vector<T> &rhs) 
{
    if (lhs.size() != rhs.size())
        throw length_error("vectors must be same size to add");
    return lhs += rhs;
}

// Subtract number from each element of vector
template <typename T>
vector<T> operator-(vector<T> lhs, const T &rhs) 
{
    for (vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] -= rhs;
    return lhs;
}

// Add number to each element of vector
template <typename T>
vector<T> operator+(vector<T> lhs, const T &rhs) 
{
    for (vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] += rhs;
    return lhs;
}


// See if number is less than each element of vector
template <typename T>
bool operator<(vector<T> lhs, const T &rhs) 
{
    bool lessThan = true;
    for (vector<double>::size_type i = 0; i < lhs.size(); ++i)
        if (lhs[i] > rhs)
        {
            lessThan = false;
        }
    return lessThan;
}

// See if number is greater than each element of vector
template <typename T>
bool operator>(vector<T> lhs, const T &rhs) 
{
    bool greaterThan = true;
    for (vector<double>::size_type i = 0; i < lhs.size(); ++i)
        if (lhs[i] < rhs)
        {
            greaterThan = false;
        }
    return greaterThan;
}

// Multiply number to each element of vector
template <typename T>
vector<T> operator*(vector<T> lhs, const T &rhs) 
{
    for (vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] *= rhs;
    return lhs;
}

// Divide each element of vector by number
template <typename T>
vector<T> operator/(vector<T> lhs, const T &rhs) 
{
    for (vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] /= rhs;
    return lhs;
}

// matrix multiplication with two-dimensional and one-dimensional array
template<typename T>
vector<T> operator*(vector<vector<T>> lhs, const vector<T> &rhs)
{
    vector<T> matrix;

    for (vector<T> row : lhs)
    {
        T newElement = 0;
        for (vector<double>::size_type i = 0; i < rhs.size(); i++)
        {
            newElement += row[i] * rhs[i];
        }     
    matrix.push_back(newElement);
    }

    return matrix;
}

template<typename T>
vector<T> abs(vector<T>& vect)
{
    for (T& value : vect)
    {
        value = abs(value);
    }
    return vect;
}

template<typename T>
T sum(vector<T>& vect)
{
    T sum = 0;
    for (T& value : vect)
    {
        sum += value;
    }
    return sum;
}

#endif /* VECTOR_OPERATIONS_H */