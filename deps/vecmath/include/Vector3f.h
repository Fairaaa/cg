#ifndef VECTOR_3F_H
#define VECTOR_3F_H

#include<random>
class Vector2f;

class Vector3f
{
public:

	static const Vector3f ZERO;
	static const Vector3f UP;
	static const Vector3f RIGHT;
	static const Vector3f FORWARD;

    Vector3f( float f = 0.f );
    Vector3f( float x, float y, float z );

	Vector3f( const Vector2f& xy, float z );
	Vector3f( float x, const Vector2f& yz );

	// copy constructors
    Vector3f( const Vector3f& rv );

	// assignment operators
    Vector3f& operator = ( const Vector3f& rv );

	// no destructor necessary

	// returns the ith element
    const float& operator [] ( int i ) const;
    float& operator [] ( int i );

    float& x();
	float& y();
	float& z();

	float x() const;
	float y() const;
	float z() const;

	Vector2f xy() const;
	Vector2f xz() const;
	Vector2f yz() const;

	Vector3f xyz() const;
	Vector3f yzx() const;
	Vector3f zxy() const;

	float length() const;
    float squaredLength() const;

	void normalize();
	Vector3f normalized() const;

	Vector2f homogenized() const;

	void negate();

	// ---- Utility ----
    operator const float* () const; // automatic type conversion for OpenGL
    operator float* (); // automatic type conversion for OpenGL 
	void print() const;	

	Vector3f& operator += ( const Vector3f& v );
	Vector3f& operator -= ( const Vector3f& v );
    Vector3f& operator *= ( float f );

    static float dot( const Vector3f& v0, const Vector3f& v1 );
	static Vector3f cross( const Vector3f& v0, const Vector3f& v1 );
    
    // computes the linear interpolation between v0 and v1 by alpha \in [0,1]
	// returns v0 * ( 1 - alpha ) * v1 * alpha
	static Vector3f lerp( const Vector3f& v0, const Vector3f& v1, float alpha );

	// computes the cubic catmull-rom interpolation between p0, p1, p2, p3
    // by t \in [0,1].  Guarantees that at t = 0, the result is p0 and
    // at p1, the result is p2.
	static Vector3f cubicInterpolate( const Vector3f& p0, const Vector3f& p1, const Vector3f& p2, const Vector3f& p3, float t );

	Vector3f clamp()
	{
		x() = x() > 1 ? 1 : x() < 0 ? 0 : x();
		y() = y() > 1 ? 1 : y() < 0 ? 0 : y();
		z() = z() > 1 ? 1 : z() < 0 ? 0 : z();
		return *this;
	}

	float getMax()
	{
		return x() > y() ? (x() > z() ? x() : z()) : (y() > z() ? y() : z());
	}

private:

	float m_elements[ 3 ];

};

// component-wise operators
Vector3f operator + ( const Vector3f& v0, const Vector3f& v1 );
Vector3f operator - ( const Vector3f& v0, const Vector3f& v1 );
Vector3f operator * ( const Vector3f& v0, const Vector3f& v1 );
Vector3f operator / ( const Vector3f& v0, const Vector3f& v1 );

// unary negation
Vector3f operator - ( const Vector3f& v );

// multiply and divide by scalar
Vector3f operator * ( float f, const Vector3f& v );
Vector3f operator * ( const Vector3f& v, float f );
Vector3f operator / ( const Vector3f& v, float f );

bool operator == ( const Vector3f& v0, const Vector3f& v1 );
bool operator != ( const Vector3f& v0, const Vector3f& v1 );

// Vector3f random3f_unit(){
// 	float x = (float)rand() / RAND_MAX;
// 	float y = (float)rand() / RAND_MAX;
// 	float z = (float)rand() / RAND_MAX;
// 	return Vector3f(x, y, z).normalized();
// }

#endif // VECTOR_3F_H
