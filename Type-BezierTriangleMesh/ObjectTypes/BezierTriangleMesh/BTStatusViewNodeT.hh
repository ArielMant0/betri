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

namespace ACG
{
namespace SceneGraph
{

template <class MeshT>
class DLLEXPORTONLY BTStatusViewNodeT : public ACG::SceneGraph::BaseNode
{
public:
    BTStatusViewNodeT(
        BaseNode* _parent,
        const std::string& _name,
        ACG::SceneGraph::SelectionNodeT<MeshT>* _statusNode,
        ACG::SceneGraph::StatusNodeT<MeshT, AreaNodeMod<MeshT> >* _areaNode,
        ACG::SceneGraph::StatusNodeT<MeshT, HandleNodeMod<MeshT> >* _handleNode,
        ACG::SceneGraph::StatusNodeT<MeshT, FeatureNodeMod<MeshT> >* _featureNode
    );

    // BaseNode implementation

    ACG_CLASSNAME(BTStatusViewNodeT);

    void draw(GLState& , const DrawModes::DrawMode& );

    void getRenderObjects(IRenderer* _renderer, GLState& _state , const DrawModes::DrawMode& _drawMode , const Material* _mat);

    void enter(GLState& _state, const DrawModes::DrawMode& _drawmode);
    void leave(GLState& _state, const DrawModes::DrawMode& _drawmode);

private:

    ACG::SceneGraph::SelectionNodeT<MeshT>*                       statusNode_;
    ACG::SceneGraph::StatusNodeT<MeshT, AreaNodeMod<MeshT> >*     areaNode_;
    ACG::SceneGraph::StatusNodeT<MeshT, HandleNodeMod<MeshT> >*   handleNode_;
    ACG::SceneGraph::StatusNodeT<MeshT, FeatureNodeMod<MeshT> >*  featureNode_;

    // visibility of nodes
    bool statusNodeVis_;
    bool areaNodeVis_;
    bool handleNodeVis_;
    bool featureNodeVis_;
};

} // namespace SceneGraph
} // namespace ACG

//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(BTSTATUSVIEWNODE_C)
#define BT_STATUS_VIEW_NODE_TEMPLATES
#include "BTStatusViewNodeT_impl.hh"
#endif
