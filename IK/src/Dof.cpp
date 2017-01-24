
#include "Dof.h"


#include <fstream>
using namespace std;


void DofList::UpdateDofs()
{
  for(int i = 0; i < mDofs.size(); i++)
    mDofs[i]->SetTransformValue(mDofs[i]->mVal);
}

void DofList::SetDofs(Vecd &data)
{	
  for( int i = 0; i < mDofs.size(); i++ ){
    mDofs[i]->mVal = data[i];
    mDofs[i]->SetTransformValue(data[i]);
  }
}

void DofList::SetDof(int index, double value)
{	
  mDofs[index]->mVal = value;
  mDofs[index]->SetTransformValue(value);
}

void DofList::GetDofs(Vecd *data)
{
  for(int i = 0; i < mDofs.size(); i++)
    (*data)[i] = mDofs[i]->GetTransformValue();
}

double DofList::GetDof(int index)
{
    return mDofs[index]->GetTransformValue();
}
