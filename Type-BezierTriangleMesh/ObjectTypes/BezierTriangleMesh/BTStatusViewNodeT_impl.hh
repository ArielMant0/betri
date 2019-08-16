//=============================================================================
//
//  StatusViewNode: Combined rendering of multiple Status Nodes
//
//=============================================================================

#ifndef BTSTATUSVIEWNODE_C
#define BTSTATUSVIEWNODE_C

//== INCLUDES =================================================================

#include "BTStatusViewNodeT.hh"
#include <ACG/Scenegraph/DrawModes.hh>

namespace ACG
{
namespace SceneGraph
{

//== IMPLEMENTATION ==========================================================

template<class MeshT>
BTStatusViewNodeT<MeshT>::BTStatusViewNodeT (
    BaseNode* _parent,
    const std::string& _name,
    ACG::SceneGraph::SelectionNodeT<MeshT>* _statusNode,
    ACG::SceneGraph::StatusNodeT<MeshT, AreaNodeMod<MeshT> >* _areaNode,
    ACG::SceneGraph::StatusNodeT<MeshT, HandleNodeMod<MeshT> >* _handleNode,
    ACG::SceneGraph::StatusNodeT<MeshT, FeatureNodeMod<MeshT> >* _featureNode
) :
    BaseNode(_parent, _name),
    statusNode_(_statusNode),
    areaNode_(_areaNode),
    handleNode_(_handleNode),
    featureNode_(_featureNode),
    statusNodeVis_(false),
    areaNodeVis_(false),
    handleNodeVis_(false),
    featureNodeVis_(false) {}

//----------------------------------------------------------------------------

template <class MeshT>
void BTStatusViewNodeT<MeshT>::draw(GLState& _state, const DrawModes::DrawMode& _drawMode)
{
    // extract all layers from drawmode
    DrawModes::DrawMode singleLayers[4]; // polygon, edge, halfedge, point

    // remove default property layer
    for (int i = 0; i < 4; ++i)
        singleLayers[i].removeLayer(0u);

    for (unsigned int i = 0; i < _drawMode.getNumLayers(); ++i) {
        const DrawModes::DrawModeProperties* props = _drawMode.getLayer(i);

        switch (props->primitive())
        {
            case DrawModes::PRIMITIVE_POLYGON:
                singleLayers[0].setDrawModeProperties(props); break;
            case DrawModes::PRIMITIVE_EDGE:
                singleLayers[1].setDrawModeProperties(props); break;
            case DrawModes::PRIMITIVE_HALFEDGE:
                singleLayers[2].setDrawModeProperties(props); break;
            case DrawModes::PRIMITIVE_POINT:
                singleLayers[3].setDrawModeProperties(props); break;
            default: break;
        }
    }

    // rendering order: faces -> edges -> halfedges -> points
    //   area -> handle -> feature -> selections
    for (int i = 0; i < 4; ++i) {

        if (singleLayers[i].getNumLayers() == 0) {
            DrawModes::DrawModeProperties defaultProps;

            switch (i) {
                case 0: defaultProps.primitive(DrawModes::PRIMITIVE_POLYGON); break;
                case 1: defaultProps.primitive(DrawModes::PRIMITIVE_EDGE); break;
                case 2: defaultProps.primitive(DrawModes::PRIMITIVE_HALFEDGE); break;
                case 3: defaultProps.primitive(DrawModes::PRIMITIVE_POINT); break;
                default: break;
            }

            singleLayers[i].addLayer(&defaultProps);
        }

        // call enter() and leave() to setup the correct material

        if (areaNodeVis_) {
            areaNode_->enter(_state, singleLayers[i]);
            areaNode_->draw(_state, singleLayers[i]);
            areaNode_->leave(_state, singleLayers[i]);
        }
        if (handleNodeVis_) {
            handleNode_->enter(_state, singleLayers[i]);
            handleNode_->draw(_state, singleLayers[i]);
            handleNode_->leave(_state, singleLayers[i]);
        }
        if (featureNodeVis_) {
            featureNode_->enter(_state, singleLayers[i]);
            featureNode_->draw(_state, singleLayers[i]);
            featureNode_->leave(_state, singleLayers[i]);
        }
        if (statusNodeVis_) {
            statusNode_->enter(_state, singleLayers[i]);
            statusNode_->draw(_state, singleLayers[i]);
            statusNode_->leave(_state, singleLayers[i]);
        }
    }
}

//----------------------------------------------------------------------------

template <class MeshT>
void BTStatusViewNodeT<MeshT>::getRenderObjects(
    IRenderer* _renderer,
    GLState& _state,
    const DrawModes::DrawMode& _drawMode,
    const Material* _mat)
{
    // extract all layers from drawmode

    DrawModes::DrawMode singleLayers[4]; // polygon, edge, halfedge, point

    // remove default property layer
    for (int i = 0; i < 4; ++i)
        singleLayers[i].removeLayer(0u);

    for (unsigned int i = 0; i < _drawMode.getNumLayers(); ++i) {
        const DrawModes::DrawModeProperties* props = _drawMode.getLayer(i);

        switch (props->primitive()) {
            case DrawModes::PRIMITIVE_POLYGON:
                singleLayers[0].setDrawModeProperties(props); break;
            case DrawModes::PRIMITIVE_EDGE:
                singleLayers[1].setDrawModeProperties(props); break;
            case DrawModes::PRIMITIVE_HALFEDGE:
                singleLayers[2].setDrawModeProperties(props); break;
            case DrawModes::PRIMITIVE_POINT:
                singleLayers[3].setDrawModeProperties(props); break;
            default: break;
        }
    }

    // rendering order: faces -> edges -> halfedges -> points
    //   area -> handle -> feature -> selections
    // rendering order only depends on priority in render objects
    for (int i = 0; i < 4; ++i) {

        if (singleLayers[i].getNumLayers() == 0) {
            DrawModes::DrawModeProperties defaultProps;

            switch (i) {
                case 0: defaultProps.primitive(DrawModes::PRIMITIVE_POLYGON); break;
                case 1: defaultProps.primitive(DrawModes::PRIMITIVE_EDGE); break;
                case 2: defaultProps.primitive(DrawModes::PRIMITIVE_HALFEDGE); break;
                case 3: defaultProps.primitive(DrawModes::PRIMITIVE_POINT); break;
                default: break;
            }

            singleLayers[i].addLayer(&defaultProps);
        }

        if (statusNodeVis_)
            statusNode_->getRenderObjects(_renderer, _state, singleLayers[i], _mat);

        if (areaNodeVis_)
            areaNode_->getRenderObjects(_renderer, _state, singleLayers[i], _mat);

        if (handleNodeVis_)
            handleNode_->getRenderObjects(_renderer, _state, singleLayers[i], _mat);

        if (featureNodeVis_)
            featureNode_->getRenderObjects(_renderer, _state, singleLayers[i], _mat);
    }
}


//----------------------------------------------------------------------------

template <class MeshT>
void BTStatusViewNodeT<MeshT>::enter( GLState& _state, const DrawModes::DrawMode& _drawmode )
{
    statusNodeVis_ = statusNode_ && statusNode_->visible();
    areaNodeVis_ = areaNode_ && areaNode_->visible();
    handleNodeVis_ = handleNode_ && handleNode_->visible();
    featureNodeVis_ = featureNode_ && featureNode_->visible();

    // these nodes are manually rendered in this StatusViewNode
    //  hide nodes to prevent drawing them again in the same draw traversal

    if (statusNodeVis_)
        statusNode_->hide();
    if (areaNodeVis_)
        areaNode_->hide();
    if (handleNodeVis_)
        handleNode_->hide();
    if (featureNodeVis_)
        featureNode_->hide();
}

//----------------------------------------------------------------------------

template <class MeshT>
void BTStatusViewNodeT<MeshT>::leave(GLState& _state, const DrawModes::DrawMode& _drawmode)
{
    if (statusNodeVis_)
        statusNode_->show();
    if (areaNodeVis_)
        areaNode_->show();
    if (handleNodeVis_)
        handleNode_->show();
    if (featureNodeVis_)
        featureNode_->show();
}

}
}

#endif // BTSTATUSVIEWNODE_C
