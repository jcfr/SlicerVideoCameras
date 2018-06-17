/*==============================================================================

  Program: 3D Slicer

  Portions (c) Copyright Brigham and Women's Hospital (BWH) All Rights Reserved.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

==============================================================================*/

// Qt includes
#include <QDebug>

// SlicerQt includes
#include "qSlicerCameraModuleWidget.h"
#include "ui_qSlicerCameraModuleWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerCameraModuleWidgetPrivate: public Ui_qSlicerCameraModuleWidget
{
public:
  qSlicerCameraModuleWidgetPrivate();
};

//-----------------------------------------------------------------------------
// qSlicerCameraModuleWidgetPrivate methods

//-----------------------------------------------------------------------------
qSlicerCameraModuleWidgetPrivate::qSlicerCameraModuleWidgetPrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerCameraModuleWidget methods

//-----------------------------------------------------------------------------
qSlicerCameraModuleWidget::qSlicerCameraModuleWidget(QWidget* _parent)
  : Superclass( _parent )
  , d_ptr( new qSlicerCameraModuleWidgetPrivate )
{
}

//-----------------------------------------------------------------------------
qSlicerCameraModuleWidget::~qSlicerCameraModuleWidget()
{
}

//-----------------------------------------------------------------------------
void qSlicerCameraModuleWidget::setup()
{
  Q_D(qSlicerCameraModuleWidget);
  d->setupUi(this);
  this->Superclass::setup();
}