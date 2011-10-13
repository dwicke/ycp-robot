// HOG-Man - Hierarchical Optimization for Pose Graphs on Manifolds
// Copyright (C) 2010 G. Grisetti, R. Kümmerle, C. Stachniss
// 
// HOG-Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// HOG-Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.


template <int Rows, int Cols, typename Base>
_Matrix<Rows,Cols,Base>& _Matrix<Rows, Cols, Base>::operator += (const _Matrix<Rows, Cols, Base>& v){
  for (int i=0; i<rows(); i++)
    for (int j=0; j<cols(); j++)
      _allocator[i][j]+=v._allocator[i][j];
  return *this;
}

template <int Rows, int Cols, typename Base>
  _Matrix<Rows,Cols,Base>& _Matrix<Rows,Cols,Base>::operator -= (const _Matrix<Rows,Cols,Base>& v){
  for (int i=0; i<rows(); i++)
    for (int j=0; j<cols(); j++)
      _allocator[i][j]-=v._allocator[i][j];
  return *this;
}

template <int Rows, int Cols, typename Base>
_Matrix<Rows,Cols,Base> _Matrix<Rows, Cols, Base>::operator - (const _Matrix<Rows, Cols, Base>& v) const {
  _Matrix<Rows, Cols, Base> aux(*this);
  aux-=v;
  return aux;
}

template <int Rows, int Cols, typename Base>
  _Matrix<Rows,Cols,Base> _Matrix<Rows, Cols, Base>::operator + (const _Matrix<Rows, Cols, Base>& v) const {
  _Matrix<Rows, Cols, Base> aux(*this);
  aux+=v;
  return aux;
}

template <int Rows, int Cols, typename Base>
  _Matrix<Rows,Cols,Base>& _Matrix<Rows,Cols,Base>::operator *= (Base c){
  for (int i=0; i<rows(); i++)
    for (int j=0; j<cols(); j++)
      _allocator[i][j]*=c;
  return *this;
}

template <int Rows, int Cols, typename Base>
  _Matrix<Rows,Cols,Base> _Matrix<Rows,Cols,Base>::operator * (Base c) const{
  _Matrix<Rows,Cols,Base> aux(*this);
  aux*=c;
  return aux;
}


template <int Rows, int Cols, typename Base>
_Matrix<Cols, Rows, Base> _Matrix<Rows, Cols, Base>::transpose() const{
  _Matrix<Cols, Rows, Base> aux(cols(),rows());
  for (int i=0; i<rows(); i++)
    for (int j=0; j<cols(); j++)
      aux._allocator[j][i]=_allocator[i][j];
  return aux;
}

template <int Rows, int Cols, typename Base>
_Matrix<Cols, Rows, Base>& _Matrix<Rows, Cols, Base>::transposeInPlace(){
  if (rows() == cols()) {
    for (int i=0; i<rows(); i++)
      for (int j=i+1; j<cols(); j++)
        swap((*this)[i][j], (*this)[j][i]);
  } else {
    _Matrix<Cols, Rows, Base> aux(cols(), rows());
    for (int i=0; i<rows(); i++)
      for (int j=0; j<cols(); j++)
        aux._allocator[j][i]=_allocator[i][j];
    *this=aux;
  }
  return *this;
}



template <int Rows, int Cols, typename Base>
_Matrix<Rows, Rows, Base> _Matrix<Rows, Cols, Base>::eye(Base factor, int size_){
  _Matrix<Rows, Rows, Base> aux(size_, size_);
  for (int i=0; i<aux.rows(); i++)
    for (int j=0; j<aux.cols(); j++)
      aux._allocator[i][j]=Base(0);
  for (int i=0; i<aux.rows(); i++){
    aux._allocator[i][i]=Base(factor);
  }
  return aux;
}

template <int Rows, int Cols, typename Base>
_Matrix<Rows, Rows, Base> _Matrix<Rows, Cols, Base>::diag(const _Vector<Rows, Base>& v){
  _Matrix<Rows, Rows, Base> aux(v.size(), v.size());
  for (int i=0; i<aux.rows(); i++)
    for (int j=0; j<aux.cols(); j++)
      aux._allocator[i][j]=Base(0);
  for (int i=0; i<aux.rows(); i++){
    aux._allocator[i][i]=v[i];
  }
  return aux;
}

template <int Rows, int Cols, typename Base>
_Matrix<Rows, Rows, Base> _Matrix<Rows, Cols, Base>::permutation(const _Vector<Rows, int>& p){
  _Matrix<Rows, Rows, Base> aux(p.size(), p.size());
  for (int i=0; i<aux.rows(); i++)
    for (int j=0; j<aux.cols(); j++)
      aux._allocator[i][j]=Base(0);
  for (int i=0; i<aux.rows(); i++){
    aux._allocator[p[i]][i]=p[i];
  }
  return aux;
}

template <int Rows, int Cols, typename Base>
_Matrix<Rows, Rows, Base> _Matrix<Rows, Cols, Base>::outerProduct(const _Vector<Rows, Base>& v1, const _Vector<Rows, Base>& v2){
  assert(v1.size()==v2.size());
  _Matrix<Rows, Rows, Base> aux(v1.size(), v1.size());
  for (int i=0; i<aux.rows(); i++)
    for (int j=0; j<aux.cols(); j++)
      aux._allocator[i][j]=v1[i]*v2[j];
  return aux;
}



template <int Rows, int Cols, typename Base>
    template <int Rows1, int Cols1>
_Matrix<Rows, Cols1, Base> _Matrix<Rows, Cols, Base>::operator*(const _Matrix<Rows1, Cols1, Base>& m) const{
  _Matrix<Cols1, Rows1, Base> tm=m.transpose(); // this makes evetyrhing cache friendly
  _Matrix<Rows, Cols1, Base> aux(rows(),m.cols());
  for (int i=0; i<aux.rows(); i++)
    for (int j=0; j<aux.cols(); j++){
      Base acc(0);
      for (int k=0; k<cols(); k++){
	acc+=_allocator[i][k]*tm[j][k];
      }
      aux._allocator[i][j]=acc;
    }
  return aux;
}

template <int Rows, int Cols, typename Base>
_Matrix<Rows, Cols, Base>& _Matrix<Rows, Cols, Base>::operator*=(const _Matrix<Rows, Cols, Base>& m) {
  assert(rows()==cols());
  *this=(*this)*m;
  return *this;
}



template <int Rows, int Cols, typename Base>
  void _Matrix<Rows, Cols, Base>::swapRows(int r1, int r2){
  for (int i=0; i<cols(); i++){
    Base aux=_allocator[r1][i];
    _allocator[r1][i]=_allocator[r2][i];
    _allocator[r2][i]=aux;
  }
}

template <int Rows, int Cols, typename Base>
  void _Matrix<Rows, Cols, Base>::sumRow(int dest, Base destFactor, int src, Base srcFactor){
  for (int i=0; i<cols(); i++){
    _allocator[dest][i]=_allocator[dest][i]*destFactor+_allocator[src][i]*srcFactor;
  }
}

template <int Rows, int Cols, typename Base>
  void _Matrix<Rows, Cols, Base>::multRow(int dest, Base destFactor){
  for (int i=0; i<cols(); i++){
    _allocator[dest][i]*=destFactor;
  }
}


template <int Rows, int Cols, typename Base>
_Matrix<Rows, Rows, Base> _Matrix<Rows, Cols, Base>::inverse() const {
  assert(rows()==cols() && "Matrix not square");


  if(rows()==2){
    _Matrix<Rows, Rows, Base> ret(*this);
    Base d=det();
    if (fabs(d)<=Base(0)){
      std::cerr << *this << std::endl;
      assert(0 && "Matrix not invertible");
    }
    ret[0][0]=_allocator[1][1];
    ret[1][1]=_allocator[0][0];
    ret[1][0]=-_allocator[1][0];
    ret[0][1]=-_allocator[0][1];
    ret*=(Base(1.)/d);
    return ret;
  }

  _Matrix<Rows, Rows, Base> L(*this);
  _Matrix<Rows, Rows, Base> U=_Matrix<Rows, Rows, Base>::eye(Base(1),rows());

  
  for (int i=0;i<rows();i++) {
    //pivoting
    Base absMax=Base(0);
    int k=-1;
    for (int q=i;q<rows();q++){
      Base absVal=fabs(L._allocator[q][i]);
      if (absVal>absMax){
	k=q;
	absMax=absVal;
      }
    }

    if (k==-1) {
      std::cerr << "Matrix not invertible" << std::endl;
      std::cerr << *this << std::endl;
      assert(0 && "Matrix not invertible");
    } else {
      L.swapRows(k,i);
      U.swapRows(k,i);
    }
    
    Base val=L._allocator[i][i];
    val=Base(1)/val;
    L.multRow(i,val);
    U.multRow(i,val);
    
    for (int j=0;j<rows();j++)
      if (j!=i) {
	Base tmp=-L._allocator[j][i];
	L.sumRow(j,Base(1),i,tmp);
	U.sumRow(j,Base(1),i,tmp);
      }
  }
  return U;
}

template <int Rows, int Cols, typename Base>
_Matrix<Rows, Rows, Base> _Matrix<Rows, Cols, Base>::cholesky() const {
  assert(Rows==Cols);
  _Matrix<Rows, Rows, Base> L=_Matrix<Rows, Rows, Base>::eye(0.,rows());
  Base diag[rows()];
  for (int i=0; i<rows(); i++)
    for (int j=0; j<=i; j++){
      if (i==j){
	Base aux=_allocator[j][j];
	for (int k=0; k<j; k++)
	  aux-=L[i][k]*L[i][k];
	assert (aux>Base(0));
	L[j][j]=sqrt(aux);
	diag[j]=1./L[j][j];
      } else { 
	Base aux=_allocator[i][j];
	for (int k=0; k<j; k++)
	  aux-=L[i][k]*L[j][k];
	L[i][j]=aux*diag[j];
      }
    }
  return L;
}

template <int Rows, int Cols, typename Base>
Base _Matrix<Rows, Cols, Base>::det() const {
  assert(Rows==Cols);

  if (rows()==2 && cols()==2){
    return _allocator[0][0]*_allocator[1][1]-_allocator[1][0]*_allocator[0][1];
  }


  _Matrix<Rows, Rows, Base> aux(*this);
  Base d=Base(1);
  for (int i=0;i<rows();i++) {
    int k=i;
    for (;k<Rows&&aux[k][i]==Base(0);k++) {}
    if (k>=Rows) return Base(0);
    Base val=aux[k][i];
    for (int j=0;j<rows();j++) {
      aux[k][j]/=val;
    }
    d=d*val;
    if (k!=i) {
      for (int j=0;j<rows();j++) {
	Base tmp=aux[k][j];
	aux[k][j]=aux[i][j];
	aux[i][j]=tmp;
      }
      d=-d;	
    }
    for (int j=i+1;j<rows();j++){
      Base tmp=aux[j][i];
      if (!(tmp==Base(0)) ){
	for (int l=0;l<rows();l++) {
	  aux[j][l]=aux[j][l]-tmp*aux[i][l];
	}
      }		
    }
  }
  return d;
}

template <int Rows, int Cols, typename Base>
int _Matrix<Rows, Cols, Base>::nullSpace(_Matrix<Rows, Cols, Base>& null_, Base epsilon) const{

  // reduce the matrix to triangular form
  _Matrix<Rows, Cols, Base> L(*this);
  bool freeVars[rows()];
  for (int i=0; i<rows(); i++)
    freeVars[i]=false;

  int fidx=0;
  for (int i=0;i<rows();i++) {
    //pivoting
    Base absMax=epsilon;
    int k=-1;
    for (int q=i;q<rows();q++){
      Base absVal=fabs(L[q][fidx]);
      if (absVal>absMax){
	k=q;
	absMax=absVal;
      }
    }

    // seek for free variables and do a pivoting for proper conditioning
    if (k==-1) {
      freeVars[fidx]=true;
      fidx++;
      continue;
    } else {
      L.swapRows(k,i);
    }
    
    // clear the bounded vars of the matrix in this column
    L.multRow(i,Base(1)/L._allocator[i][fidx]);
    for (int j=0;j<rows();j++)
      if (j!=i)
	L.sumRow(j,Base(1),i,-L[j][fidx]);
    fidx++;
  }

  while (fidx<cols()){
    freeVars[fidx]=true;
    fidx++;
  }
  null_.fill(Base(0));
  int fv=0;
  for (int i=0; i<cols(); i++){
    if (freeVars[i]){
      int bv=0;
      for (int j=0; j<i; j++){
	if (! freeVars[j]){
	  null_[fv][j]=-L[bv][i];
	  bv++;
	}
      }
      null_[fv][i]=Base(1);
      fv++;
    }
  }
  return fv;
}

template <int R, int C, typename Base>
std::ostream& operator << (std::ostream& os, const _Matrix<R, C, Base>& m){
  for (int j=0; j<m.rows(); j++){
    if (j > 0)
      os << std::endl;
    for (int i=0; i<m.cols(); i++){
      if (i>0)
        os << " ";
      os << (m[j][i]);
    }
  }
  return os;
}

template <int Rows, int Cols, typename Base>
_Vector<Rows, Base> _Matrix<Rows, Cols, Base>::operator* (const _Vector<Cols, Base>& v) const{
  _Vector<Rows, Base> aux;
  for (int i=0; i<rows(); i++){
    Base acc=Base(0);
    for (int j=0; j<cols(); j++)
      acc+=_allocator[i][j]*v[j];
    aux[i]=acc;
  }
  return aux;
}

template <int N, typename Base>
_Vector<N, Base>::operator _Matrix<N, 1, Base> () const{
  _Matrix<N, 1, Base> aux;
  for(int i=0; i<size(); i++)
    aux[0][i]=_allocator[i];
  return aux;
}

template <int Rows, int Cols, typename Base>
void _Matrix<Rows, Cols, Base>::fill(Base scalar)
{
  for (int j=0; j<rows(); j++)
    for (int i=0; i<cols(); i++)
      (*this)[j][i] = scalar;
}

template <int R, int C, typename Base>
_Matrix<R, C, Base> operator* (Base x, const _Matrix<R, C, Base>& m)
{
  return m * x;
}
