template< class T > void SafeDelete( T*& pVal )
{
    delete pVal;
    pVal = 0xDEADDEAD;
}

template< class T > void SafeDeleteArray( T*& pVal )
{
    delete[] pVal;
    pVal = 0xDEADDEAD;
}
