/*=auto=========================================================================

Portions (c) Copyright 2018 Robarts Research Institute. All Rights Reserved.

See COPYRIGHT.txt
or http://www.slicer.org/copyright/copyright.txt for details.

Program:   3D Slicer
Module:    $RCSfile: vtkSlicerVideoCamerasLogic.h,v $
Date:      $Date: 2018/6/16 10:54:09 $
Version:   $Revision: 1.0 $

=========================================================================auto=*/

// .NAME vtkSlicerVideoCamerasLogic - slicer logic class for volumes manipulation
// .SECTION Description
// This class manages the logic associated with reading, saving,
// and changing propertied of the volumes


#ifndef __vtkSlicerVideoCamerasLogic_h
#define __vtkSlicerVideoCamerasLogic_h

// Video camera includes
#include <vtkMRMLVideoCameraNode.h>

// Slicer includes
#include <vtkSlicerModuleLogic.h>
#include <vtkMRMLScalarVolumeNode.h>
#include <vtkMRMLLinearTransformNode.h>

// MRML includes

// STD includes
#include <array>
#include <queue>

#include "vtkSlicerVideoCamerasModuleLogicExport.h"

class vtkMultiThreader;
class vtkMutexLock;

class VTK_SLICER_VIDEOCAMERAS_MODULE_LOGIC_EXPORT vtkSlicerVideoCamerasAutomaticSegmentationResult : public vtkObject
{
public:
  static vtkSlicerVideoCamerasAutomaticSegmentationResult* New();
  vtkTypeMacro(vtkSlicerVideoCamerasAutomaticSegmentationResult, vtkObject);

  float       CenterX;
  float       CenterY;
  float       Radius;
  float       Tip_Camera[3];
};

/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_VIDEOCAMERAS_MODULE_LOGIC_EXPORT vtkSlicerVideoCamerasLogic :
  public vtkSlicerModuleLogic
{
public:
  enum VideoCameraLogicEventType
  {
    AutomaticSegmentationResultEvent = 228400
  };

  static vtkSlicerVideoCamerasLogic* New();
  vtkTypeMacro(vtkSlicerVideoCamerasLogic, vtkSlicerModuleLogic);
  void PrintSelf(ostream& os, vtkIndent indent);

  ///
  /// Add into the scene a new mrml videoCamera node and
  /// read it's properties from a specified file
  /// A storage node is also added into the scene
  vtkMRMLVideoCameraNode* AddVideoCamera(const char* filename, const char* nodeName = NULL);

  ///
  /// Threaded functionality to automatically segment a red circle from an image node (which is expected to continually change)
  void SegmentCircleInImageAsync(vtkMRMLScalarVolumeNode* volumeNode,
                                 vtkMRMLVideoCameraNode* videoCamera,
                                 vtkMRMLLinearTransformNode* tipToCameraNode,
                                 double colorRange1Low[3],
                                 double colorRange1High[3],
                                 double colorRange2Low[3],
                                 double colorRange2High[3],
                                 double minDist,
                                 double param1,
                                 double param2,
                                 int minRadius,
                                 int maxRadius);

  ///
  /// This function doesn't do anything other than fire events on the main GUI thread
  /// Must be called regularly by the GUI thread
  void PeriodicSegmentationProcess();

protected:
  vtkSlicerVideoCamerasLogic();
  virtual ~vtkSlicerVideoCamerasLogic();

  static void* SegmentImageThreadFunction(void* ptr);
  void QueueSegmentationResult(vtkSlicerVideoCamerasAutomaticSegmentationResult* result);

  virtual void SetMRMLSceneInternal(vtkMRMLScene* newScene);
  /// Register MRML Node classes to Scene. Gets called automatically when the MRMLScene is attached to this logic class.
  virtual void RegisterNodes();
  virtual void UpdateFromMRMLScene();
  virtual void OnMRMLSceneNodeAdded(vtkMRMLNode* node);
  virtual void OnMRMLSceneNodeRemoved(vtkMRMLNode* node);

protected:
  class vtkAutoSegmentationParameters : public vtkObject
  {
  public:
    vtkSetObjectMacro(AutomaticSegmentationCameraNode, vtkMRMLVideoCameraNode);
    vtkSetObjectMacro(AutomaticSegmentationImageNode, vtkMRMLScalarVolumeNode);
    vtkSetObjectMacro(TipToCameraTransformNode, vtkMRMLLinearTransformNode);

    static vtkAutoSegmentationParameters* New();
    vtkTypeMacro(vtkAutoSegmentationParameters, vtkObject);

    vtkMRMLScalarVolumeNode*              AutomaticSegmentationImageNode;
    vtkMRMLVideoCameraNode*               AutomaticSegmentationCameraNode;
    vtkMRMLLinearTransformNode*           TipToCameraTransformNode;
    std::array<std::array<double, 3>, 4>  ColorRanges;
    double                                MinDist;
    double                                Param1;
    double                                Param2;
    int                                   MinRadius;
    int                                   MaxRadius;
    vtkSlicerVideoCamerasLogic*           ParentLogic;
  };

  vtkSmartPointer<vtkMultiThreader>                               Threader;
  int                                                             ThreadID;
  vtkSmartPointer<vtkMutexLock>                                   ThreadMutexLock;
  vtkSmartPointer<vtkMutexLock>                                   EventQueueMutex;
  std::queue<vtkSlicerVideoCamerasAutomaticSegmentationResult*>   ResultQueue;

private:
  vtkSlicerVideoCamerasLogic(const vtkSlicerVideoCamerasLogic&); // Not implemented
  void operator=(const vtkSlicerVideoCamerasLogic&); // Not implemented
};

#endif
