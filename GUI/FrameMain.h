#ifndef __FrameMain__
#define __FrameMain__

/**
 @file
 Subclass of BaseFrameMain, which is generated by wxFormBuilder.
 */

#include "GLAxis.h"
#include "GLGridPlane.h"
#include "GUI.h"
#include "Util/ColorMapFactory.h"
#include "Util/ObjIO.h"

#include <fstream>
#include <list>
#include <map>
#include <sstream>
#include <typeinfo>

#ifdef LAB1
#include "Geometry/HalfEdgeMesh.h"
#include "Geometry/SimpleMesh.h"
#endif  // Lab1

#ifdef LAB2
#include "Decimation/QuadricDecimationMesh.h"
#include "Decimation/SimpleDecimationMesh.h"
#endif  // Lab2

#ifdef LAB3
#include "Subdivision/LoopSubdivisionMesh.h"
#include "Subdivision/StrangeSubdivisionMesh.h"
#include "Subdivision/UniformCubicSpline.h"
#include "Subdivision/UniformCubicSplineSubdivisionCurve.h"
#endif  // Lab3

#ifdef LAB4
#include "Geometry/CSG.h"
#include "Geometry/Cube.h"
#include "Geometry/ImplicitGradientField.h"
#include "Geometry/ImplicitMesh.h"
#include "Geometry/ImplicitValueField.h"
#include "Geometry/Quadric.h"
#include "Geometry/Sphere.h"
#include "ScalarCutPlane.h"
#include "VectorCutPlane.h"
#endif  // Lab4

#ifdef LAB5
#include "Levelset/LevelSet.h"
#include "Levelset/OperatorAdvect.h"
#include "Levelset/OperatorDilateErode.h"
#include "Levelset/OperatorMeanCurvatureFlow.h"
#include "Levelset/OperatorMorph.h"
#include "Levelset/OperatorReinitialize.h"
#include "Levelset/OperatorReinitializeFastMarching.h"
#include "Math/ConstantVectorField.h"
#include "Math/VortexVectorField.h"
#endif  // Lab5

#ifdef LAB6
#include "Fluid/FluidSolver.h"
#include "FluidVoxelCutPlane.h"
#include "Util/GLObjectPlayback.h"
#endif  // Lab6

/** Implementing BaseFrameMain */
class FrameMain : public BaseFrameMain {
public:
    /** Constructor */
    FrameMain(wxWindow* parent);
    static size_t numScreenCaptures;

protected:
    class VisualizationModeData : public wxClientData {
    public:
        VisualizationModeData(const GLObject::VisualizationMode& VisualizationMode)
            : mVisualizationMode(VisualizationMode) {}

        const GLObject::VisualizationMode& GetVisualizationMode() { return mVisualizationMode; }

    protected:
        GLObject::VisualizationMode mVisualizationMode;
    };

    std::map<const char*, std::list<wxWindow*> > mPanelSwitches;
    std::map<std::string, std::list<std::string> > mDependentObjects;

    // Window close in MSW_XP (both VS2005/2008) is not working without this
#ifdef WIN32
    virtual void OnClose(wxCloseEvent& event) {
        if (event.CanVeto()) wxWindow::Destroy();
    }
#endif

    void AddUniqueObject(GLObject* object);
    void RemoveObject(GLObject* object);
    void DeleteObjects(wxCommandEvent& event);
    void DeleteDependentObjects(GLObject* object);
    void UpdateDependentObjects(GLObject* object);
    void ObjectSelected(wxCommandEvent& event);
    void SelectObjects(wxCommandEvent& event);
    void MoveObjectsUp(wxCommandEvent& event);
    void MoveObjecsDown(wxCommandEvent& event);
    void TextCtrlFocus(wxFocusEvent& event);
    void TransformObjects(wxCommandEvent& event);
    void VisualizeWireframe(wxCommandEvent& event);
    void VisualizeMeshNormals(wxCommandEvent& event);
    void OpacityChanged(wxScrollEvent& event);
    void ApplyColormap(wxCommandEvent& event);
    void SetVisualizationMode(wxCommandEvent& event);
    void ScaleChanged(wxCommandEvent& event);
    void ToggleUniformScaling(wxCommandEvent& event);
    void ToggleAutoMinMax(wxCommandEvent& event);
    void SaveMesh(wxCommandEvent& event);
    void CaptureScreen(wxCommandEvent& event);
    void Dilate(wxCommandEvent& event);
    void Erode(wxCommandEvent& event);
    void Smooth(wxCommandEvent& event);
    double GetAmount();
    void HideAllPanels();
    void UpdatePanels();

#ifdef LAB1
    void AddObjectSimpleMesh(wxCommandEvent& event);
    void AddObjectHalfEdgeMesh(wxCommandEvent& event);
    template <class MeshType>
    MeshType* AddMesh(const wxString& path);
#endif  // Lab1

#ifdef LAB2
    void AddObjectSimpleDecimationMesh(wxCommandEvent& event);
    void AddObjectQuadricDecimationMesh(wxCommandEvent& event);
    void DecimateObjects(wxCommandEvent& event);
#endif  // Lab2

#ifdef LAB3
    void AddObjectCubicSpline(wxCommandEvent& event);
    void AddObjectSubdivisionCurve(wxCommandEvent& event);
    void AddObjectLoopSubdivisionMesh(wxCommandEvent& event);
    void AddObjectStrangeSubdivisionMesh(wxCommandEvent& event);
    void SubdivideObjects(wxCommandEvent& event);
#endif  // Lab3

#ifdef LAB4
    void AddObjectImplicitSphere(wxCommandEvent& event);
    void AddObjectImplicitMesh(wxCommandEvent& event);
    void AddObjectQuadricPlane(wxCommandEvent& event);
    void AddObjectQuadricCylinder(wxCommandEvent& event);
    void AddObjectQuadricEllipsoid(wxCommandEvent& event);
    void AddObjectQuadricCone(wxCommandEvent& event);
    void AddObjectQuadricParaboloid(wxCommandEvent& event);
    void AddObjectQuadricHyperboloid(wxCommandEvent& event);
    void AddObjectScalarCutPlane(wxCommandEvent& event);
    void AddObjectVectorCutPlane(wxCommandEvent& event);
    void Union(wxCommandEvent& event);
    void Intersection(wxCommandEvent& event);
    void Difference(wxCommandEvent& event);
    void SwitchBlending(wxCommandEvent& event);
    void ResampleImplicit(wxCommandEvent& event);
    void DifferentialScaleChanged(wxScrollEvent& event);
    double GetMeshSampling();
    double GetDifferentialScale();
    template <class CSGType, class CSGTypeBlend>
    Implicit* CSG(const std::string& oper);
#endif  // Lab4

#ifdef LAB5
    void LoadLevelset(wxCommandEvent& event);
    void ConvertToLevelset(wxCommandEvent& event);
    void AddTemplate1(wxCommandEvent& event);
    void AddTemplate2(wxCommandEvent& event);
    void AddTemplate3(wxCommandEvent& event);
    void LevelsetReinitialize(wxCommandEvent& event);
    void LevelsetAdvect(wxCommandEvent& event);
    void LevelsetDilate(wxCommandEvent& event);
    void LevelsetErode(wxCommandEvent& event);
    void LevelsetSmooth(wxCommandEvent& event);
    void LevelsetMorph(wxCommandEvent& event);
    void EnableNarrowband(wxCommandEvent& event);
    double GetPropagationTime();
    long int GetIterations();
#endif  // Lab5

#ifdef LAB6
    FluidSolver* mFluidSolver;
    std::list<std::string> mFluidSolverDependentObjects;
    std::vector<SimpleMesh> mFluidSequence;
    GLObjectPlayback* mFluidPlaybackObject;

    void InitializeFluidSolver();
    void FluidSetSolid(wxCommandEvent& event);
    void FluidSetFluid(wxCommandEvent& event);
    void FluidSolve(wxCommandEvent& event);
    void FluidVisualizeVelocities(wxCommandEvent& event);
    void FluidVisualizeVoxelsClassification(wxCommandEvent& event);
    void PlaySimulation(wxCommandEvent& event);
    void SaveSimulationFrames(wxCommandEvent& event);
    void FluidPlayback(wxScrollEvent& event);
#endif  // Lab6
};

#ifdef LAB1
template <class MeshType>
MeshType* FrameMain::AddMesh(const wxString& path) {
    wxString filename = path.AfterLast('/');
    if (filename == path)  // If we're on Windows
        filename = path.AfterLast('\\');
    wxString suffix = path.AfterLast('.');

    if (suffix == _T("obj")) {
        // Create new mesh
        MeshType* mesh = new MeshType();
        mesh->SetName(std::string(filename.mb_str()));

        // Load mesh and add to geometry list
        std::ifstream infile;
        ObjIO objIO;
        infile.open(path.mb_str());
        objIO.Load(mesh, infile);
        mesh->Initialize();

        // Add mesh to scene
        AddUniqueObject(mesh);

        return mesh;
    } else
        std::cerr << "Error: File type not supported" << std::endl;

    return NULL;
}
#endif  // Lab1

#ifdef LAB4
template <class CSGType, class CSGTypeBlend>
Implicit* FrameMain::CSG(const std::string& oper) {
    std::list<GLObject*> objects = mGLViewer->GetSelectedObjects();
    std::list<GLObject*>::iterator iter = objects.begin();
    std::list<GLObject*>::iterator iend = objects.end();

    // Find first implicit in the selected list
    Implicit* impl = NULL;
    while (iter != iend) {
        impl = dynamic_cast<Implicit*>(*iter);
        iter++;
        if (impl != NULL) break;
    }

    // No implicit found - exit...
    if (impl == NULL) return NULL;

    // Else, remove implicit and do successive CSG
    RemoveObject(impl);
    while (iter != iend) {
        Implicit* second = dynamic_cast<Implicit*>(*iter);
        if (second != NULL) {
            std::string name = impl->GetName() + " " + oper + " " + second->GetName();

            if (mBlend->IsChecked()) {
                long int blend;
                if (!mBlendParameter->GetValue().ToLong(&blend)) {
                    std::cerr << "Error: can't parse CSG blend parameter - defaulting to 10"
                              << std::endl;
                    mBlendParameter->SetValue(_T("10"));
                    blend = 10;
                }
                impl = new CSGTypeBlend(impl, second, blend);
            } else
                impl = new CSGType(impl, second);

            impl->SetName(name);
            RemoveObject(second);
        }
        iter++;
    }

    return impl;
}
#endif  // Lab4

#endif  // __FrameMain__
