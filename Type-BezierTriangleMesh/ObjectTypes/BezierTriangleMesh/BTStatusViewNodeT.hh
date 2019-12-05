//=============================================================================
//
//  StatusViewNode: Combined rendering of multiple Status Nodes
//
//=============================================================================

#pragma once
//== INCLUDES =================================================================

#include <OpenFlipper/common/GlobalDefines.hh>

#include <ACG/Scenegraph/StatusNodesT.hh>

#include "BTStatusNodeMods.hh"

//== CLASS DEFINITION =========================================================

namespace betri
{
namespace vis
{

template <class MeshT>
class DLLEXPORTONLY BTStatusViewNodeT : public ACG::SceneGraph::BaseNode
{
public:
    BTStatusViewNodeT(
        BaseNode* _parent,
        const std::string& _name,
        ACG::SceneGraph::SelectionNodeT<MeshT>* _statusNode,
        ACG::SceneGraph::StatusNodeT<MeshT, betri::vis::BTAreaNodeMod<MeshT> >* _areaNode,
        ACG::SceneGraph::StatusNodeT<MeshT, betri::vis::BTHandleNodeMod<MeshT> >* _handleNode,
        ACG::SceneGraph::StatusNodeT<MeshT, betri::vis::BTFeatureNodeMod<MeshT> >* _featureNode
    );

    // BaseNode implementation

    ACG_CLASSNAME(BTStatusViewNodeT);

    void draw(ACG::GLState& , const ACG::SceneGraph::DrawModes::DrawMode& );

    void getRenderObjects(ACG::IRenderer* _renderer, ACG::GLState& _state, const ACG::SceneGraph::DrawModes::DrawMode& _drawMode , const ACG::SceneGraph::Material* _mat);

    void enter(ACG::GLState& _state, const ACG::SceneGraph::DrawModes::DrawMode& _drawmode);
    void leave(ACG::GLState& _state, const ACG::SceneGraph::DrawModes::DrawMode& _drawmode);

private:

    ACG::SceneGraph::SelectionNodeT<MeshT>*                       statusNode_;
    ACG::SceneGraph::StatusNodeT<MeshT, betri::vis::BTAreaNodeMod<MeshT> >*     areaNode_;
    ACG::SceneGraph::StatusNodeT<MeshT, betri::vis::BTHandleNodeMod<MeshT> >*   handleNode_;
    ACG::SceneGraph::StatusNodeT<MeshT, betri::vis::BTFeatureNodeMod<MeshT> >*  featureNode_;

    // visibility of nodes
    bool statusNodeVis_;
    bool areaNodeVis_;
    bool handleNodeVis_;
    bool featureNodeVis_;
};

} // namespace vis
} // namespace betri

//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(BTSTATUSVIEWNODE_C)
#define BT_STATUS_VIEW_NODE_TEMPLATES
#include "BTStatusViewNodeT_impl.hh"
#endif
