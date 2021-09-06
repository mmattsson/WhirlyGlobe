/*  SceneRendererMTL.mm
 *  WhirlyGlobeLib
 *
 *  Created by Steve Gifford on 5/16/19.
 *  Copyright 2011-2021 mousebird consulting
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#import "SceneRendererMTL.h"
#import "BasicDrawableBuilderMTL.h"
#import "BasicDrawableInstanceBuilderMTL.h"
#import "BillboardDrawableBuilderMTL.h"
#import "ScreenSpaceDrawableBuilderMTL.h"
#import "WideVectorDrawableBuilderMTL.h"
#import "RenderTargetMTL.h"
#import "DynamicTextureAtlasMTL.h"
#import "MaplyView.h"
#import "WhirlyKitLog.h"
#import "DefaultShadersMTL.h"
#import "RawData_NSData.h"
#import "RenderTargetMTL.h"

using namespace Eigen;

namespace WhirlyKit
{

WorkGroupMTL::WorkGroupMTL(GroupType inGroupType)
{
    groupType = inGroupType;
    
    switch (groupType) {
        case Calculation:
            // For calculation we don't really have a render target
            renderTargetContainers.push_back(WorkGroupMTL::makeRenderTargetContainer(NULL));
            break;
        case Offscreen:
            break;
        case ReduceOps:
            break;
        case ScreenRender:
            break;
    }
}

WorkGroupMTL::~WorkGroupMTL()
{
}

RenderTargetContainerMTL::RenderTargetContainerMTL(RenderTargetRef renderTarget)
: RenderTargetContainer(renderTarget)
{
}

RenderTargetContainerRef WorkGroupMTL::makeRenderTargetContainer(RenderTargetRef renderTarget)
{
    return RenderTargetContainerRef(new RenderTargetContainerMTL(renderTarget));
}
    
RendererFrameInfoMTL::RendererFrameInfoMTL()
{
}
    
RendererFrameInfoMTL::RendererFrameInfoMTL(const RendererFrameInfoMTL &that)
: RendererFrameInfo(that)
{
}

SceneRendererMTL::SceneRendererMTL(id<MTLDevice> mtlDevice,id<MTLLibrary> mtlLibrary, float inScale)
    : setupInfo(mtlDevice,mtlLibrary),
    _isShuttingDown(std::make_shared<bool>(false)), lastRenderNo(0), renderEvent(nil)
{
    offscreenBlendEnable = false;
    indirectRender = false;
    if (@available(iOS 13.0, *))
    {
        if ([mtlDevice supportsFeatureSet:MTLFeatureSet_iOS_GPUFamily3_v4])
        {
            indirectRender = true;
        }
    }
#if TARGET_OS_SIMULATOR
    indirectRender = false;
#endif

    init();
        
    // Calculation shaders
    workGroups.push_back(std::make_shared<WorkGroupMTL>(WorkGroup::Calculation));
    // Offscreen target render group
    workGroups.push_back(std::make_shared<WorkGroupMTL>(WorkGroup::Offscreen));
    // Middle one for weird stuff
    workGroups.push_back(std::make_shared<WorkGroupMTL>(WorkGroup::ReduceOps));
    // Last workgroup is used for on screen rendering
    workGroups.push_back(std::make_shared<WorkGroupMTL>(WorkGroup::ScreenRender));

    scale = inScale;
    setupInfo.mtlDevice = mtlDevice;
    setupInfo.uniformBuff = setupInfo.heapManage.allocateBuffer(HeapManagerMTL::Drawable,sizeof(WhirlyKitShader::Uniforms));
    setupInfo.lightingBuff = setupInfo.heapManage.allocateBuffer(HeapManagerMTL::Drawable,sizeof(WhirlyKitShader::Lighting));
    releaseQueue = dispatch_queue_create("Maply release queue", DISPATCH_QUEUE_SERIAL);
}
    
SceneRendererMTL::~SceneRendererMTL()
{
}

SceneRendererMTL::Type SceneRendererMTL::getType()
{
    return RenderMetal;
}

const RenderSetupInfo *SceneRendererMTL::getRenderSetupInfo() const
{
    return &setupInfo;
}

void SceneRendererMTL::setView(View *newView)
{
    SceneRenderer::setView(newView);
}

void SceneRendererMTL::setScene(Scene *newScene)
{
    SceneRenderer::setScene(newScene);
    
    // Slots we need to refer to on the C++ side
    slotMap[a_maskNameID] = WhirlyKitShader::WKSVertexMaskAttribute;
    for (unsigned int ii=0;ii<WhirlyKitMaxMasks;ii++)
        slotMap[a_maskNameIDs[ii]] = WhirlyKitShader::WKSVertexMaskAttribute+ii;
}

bool SceneRendererMTL::setup(int sizeX,int sizeY,bool offscreen)
{
    // Set up a default render target
    RenderTargetMTLRef defaultTarget = std::make_shared<RenderTargetMTL>(EmptyIdentity);
    defaultTarget->width = sizeX;
    defaultTarget->height = sizeY;
    defaultTarget->clearEveryFrame = false;
    if (offscreen) {
        framebufferWidth = sizeX;
        framebufferHeight = sizeY;
        
        // Create the texture we'll use right here
        TextureMTLRef fbTexMTL = std::make_shared<TextureMTL>("Framebuffer Texture");
        fbTexMTL->setWidth(sizeX);
        fbTexMTL->setHeight(sizeY);
        fbTexMTL->setIsEmptyTexture(true);
        fbTexMTL->setFormat(TexTypeUnsignedByte);
        fbTexMTL->createInRenderer(&setupInfo);
        framebufferTex = fbTexMTL;
        
        // And one for depth
        TextureMTLRef depthTexMTL = std::make_shared<TextureMTL>("Framebuffer Depth Texture");
        depthTexMTL->setWidth(sizeX);
        depthTexMTL->setHeight(sizeY);
        depthTexMTL->setIsEmptyTexture(true);
        depthTexMTL->setFormat(TexTypeDepthFloat32);
        depthTexMTL->createInRenderer(&setupInfo);

        // Note: Should make this optional
        defaultTarget->blendEnable = offscreenBlendEnable;
        defaultTarget->setTargetTexture(fbTexMTL.get());
        defaultTarget->setTargetDepthTexture(depthTexMTL.get());
    } else {
        if (sizeX > 0 && sizeY > 0)
            defaultTarget->init(this,nullptr,EmptyIdentity);
        defaultTarget->blendEnable = true;
    }
    renderTargets.push_back(defaultTarget);
    
    workGroups[WorkGroup::ScreenRender]->addRenderTarget(defaultTarget);
    
    return true;
}
    
void SceneRendererMTL::setClearColor(const RGBAColor &color)
{
    if (renderTargets.empty())
        return;
    
    auto defaultTarget = renderTargets.back();
    defaultTarget->setClearColor(color);
}

bool SceneRendererMTL::resize(int sizeX,int sizeY)
{
    // Don't want to deal with it for offscreen rendering
    if (framebufferTex)
        return false;
    
    framebufferWidth = sizeX;
    framebufferHeight = sizeY;
    
    RenderTargetRef defaultTarget = renderTargets.back();
    defaultTarget->width = sizeX;
    defaultTarget->height = sizeY;
    defaultTarget->init(this, NULL, EmptyIdentity);
    
    return true;
}
        
void SceneRendererMTL::setupUniformBuffer(RendererFrameInfoMTL *frameInfo,id<MTLBlitCommandEncoder> bltEncode,CoordSystemDisplayAdapter *coordAdapter)
{
    SceneRendererMTL *sceneRender = (SceneRendererMTL *)frameInfo->sceneRenderer;
    
    WhirlyKitShader::Uniforms uniforms;
    bzero(&uniforms,sizeof(uniforms));
    CopyIntoMtlFloat4x4Pair(uniforms.mvpMatrix,uniforms.mvpMatrixDiff,frameInfo->mvpMat4d);
    CopyIntoMtlFloat4x4(uniforms.mvpInvMatrix,frameInfo->mvpInvMat);
    CopyIntoMtlFloat4x4Pair(uniforms.mvMatrix,uniforms.mvMatrixDiff,frameInfo->viewAndModelMat4d);
    CopyIntoMtlFloat4x4(uniforms.mvNormalMatrix,frameInfo->viewModelNormalMat);
    CopyIntoMtlFloat4x4(uniforms.pMatrix,frameInfo->projMat);
    CopyIntoMtlFloat3(uniforms.eyePos,frameInfo->eyePos);
    CopyIntoMtlFloat3(uniforms.eyeVec,frameInfo->eyeVec);
    CopyIntoMtlFloat2(uniforms.screenSizeInDisplayCoords,Point2f(frameInfo->screenSizeInDisplayCoords.x(),frameInfo->screenSizeInDisplayCoords.y()));
    Point2f frameSize(frameInfo->sceneRenderer->framebufferWidth,frameInfo->sceneRenderer->framebufferHeight);
    CopyIntoMtlFloat2(uniforms.frameSize, frameSize);
    uniforms.globeMode = !coordAdapter->isFlat();
    uniforms.frameCount = frameCount;
    uniforms.currentTime = frameInfo->currentTime - scene->getBaseTime();
    for (unsigned int ii=0;ii<MaplyMaxZoomSlots;ii++) {
        frameInfo->scene->copyZoomSlots(uniforms.zoomSlots);
    }
    
    // Copy this to a buffer and then blit that buffer into place
    // TODO: Try to reuse these
    auto buff = setupInfo.heapManage.allocateBuffer(HeapManagerMTL::HeapType::Drawable, &uniforms, sizeof(uniforms));
    [bltEncode copyFromBuffer:buff.buffer sourceOffset:buff.offset toBuffer:sceneRender->setupInfo.uniformBuff.buffer destinationOffset:sceneRender->setupInfo.uniformBuff.offset size:sizeof(uniforms)];
}

void SceneRendererMTL::setupLightBuffer(SceneMTL *scene,RendererFrameInfoMTL *frameInfo,id<MTLBlitCommandEncoder> bltEncode)
{
    SceneRendererMTL *sceneRender = (SceneRendererMTL *)frameInfo->sceneRenderer;

    WhirlyKitShader::Lighting lighting;
    lighting.numLights = lights.size();
    for (unsigned int ii=0;ii<lighting.numLights;ii++) {
        DirectionalLight &dirLight = lights[ii];
        
        Eigen::Vector3f dir = dirLight.pos.normalized();
        Eigen::Vector3f halfPlane = (dir + Eigen::Vector3f(0,0,1)).normalized();
        
        WhirlyKitShader::Light &light = lighting.lights[ii];
        CopyIntoMtlFloat3(light.direction,dir);
        CopyIntoMtlFloat3(light.halfPlane,halfPlane);
        CopyIntoMtlFloat4(light.ambient,dirLight.getAmbient());
        CopyIntoMtlFloat4(light.diffuse,dirLight.getDiffuse());
        CopyIntoMtlFloat4(light.specular,dirLight.getSpecular());
        light.viewDepend = dirLight.viewDependent ? 0.0f : 1.0f;
    }
    CopyIntoMtlFloat4(lighting.mat.ambient,defaultMat.getAmbient());
    CopyIntoMtlFloat4(lighting.mat.diffuse,defaultMat.getDiffuse());
    CopyIntoMtlFloat4(lighting.mat.specular,defaultMat.getSpecular());
    lighting.mat.specularExponent = defaultMat.getSpecularExponent();
    
    // Copy this to a buffer and then blit that buffer into place
    // TODO: Try to reuse these
    auto buff = setupInfo.heapManage.allocateBuffer(HeapManagerMTL::HeapType::Drawable, &lighting, sizeof(lighting));
    [bltEncode copyFromBuffer:buff.buffer sourceOffset:buff.offset toBuffer:sceneRender->setupInfo.lightingBuff.buffer destinationOffset:sceneRender->setupInfo.lightingBuff.offset size:sizeof(lighting)];
}
    
void SceneRendererMTL::setupDrawStateA(WhirlyKitShader::UniformDrawStateA &drawState)
{
    // That was anti-climactic
    bzero(&drawState,sizeof(drawState));
    drawState.zoomSlot = -1;
}
    
MTLRenderPipelineDescriptor *SceneRendererMTL::defaultRenderPipelineState(SceneRendererMTL *sceneRender,ProgramMTL *program,RenderTargetMTL *renderTarget)
{
    MTLRenderPipelineDescriptor *renderDesc = [[MTLRenderPipelineDescriptor alloc] init];
    renderDesc.vertexFunction = program->vertFunc;
    renderDesc.fragmentFunction = program->fragFunc;
    
    renderDesc.colorAttachments[0].pixelFormat = renderTarget->getPixelFormat();
    if (renderTarget->getRenderPassDesc().depthAttachment.texture)
        renderDesc.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
    
    if (renderTarget->blendEnable) {
        renderDesc.colorAttachments[0].blendingEnabled = true;
        renderDesc.colorAttachments[0].rgbBlendOperation = MTLBlendOperationAdd;
        renderDesc.colorAttachments[0].alphaBlendOperation = MTLBlendOperationAdd;
        renderDesc.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
        renderDesc.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorSourceAlpha;
        renderDesc.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        renderDesc.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    } else {
        renderDesc.colorAttachments[0].blendingEnabled = false;
    }
    
    if (@available(iOS 13.0, *)) {
        if (indirectRender)
            renderDesc.supportIndirectCommandBuffers = true;
    }
    
    return renderDesc;
}
    
void SceneRendererMTL::addSnapshotDelegate(NSObject<WhirlyKitSnapshot> *newDelegate)
{
    snapshotDelegates.push_back(newDelegate);
}

void SceneRendererMTL::removeSnapshotDelegate(NSObject<WhirlyKitSnapshot> *oldDelegate)
{
    snapshotDelegates.erase(std::remove(snapshotDelegates.begin(), snapshotDelegates.end(), oldDelegate), snapshotDelegates.end());
}

void SceneRendererMTL::updateWorkGroups(RendererFrameInfo *inFrameInfo)
{
    RendererFrameInfoMTL *baseFrameInfo = (RendererFrameInfoMTL *)inFrameInfo;
    SceneRenderer::updateWorkGroups(baseFrameInfo);
    
    if (!indirectRender)
        return;
    
    // Build the indirect command buffers if they're available
    if (@available(iOS 13.0, *)) {
        for (const auto &workGroup : workGroups) {
            for (const auto &targetContainer : workGroup->renderTargetContainers) {
                if (targetContainer->drawables.empty() && !targetContainer->modified)
                    continue;
                RenderTargetContainerMTLRef targetContainerMTL = std::dynamic_pointer_cast<RenderTargetContainerMTL>(targetContainer);
                targetContainerMTL->drawGroups.clear();

                RenderTargetMTLRef renderTarget;
                if (!targetContainer->renderTarget) {
                    // Need some sort of render target even if we're not really rendering
                    renderTarget = std::dynamic_pointer_cast<RenderTargetMTL>(renderTargets.back());
                } else {
                    renderTarget = std::dynamic_pointer_cast<RenderTargetMTL>(targetContainer->renderTarget);
                }
                if (!renderTarget)
                {
                    continue;
                }

                // Sort the drawables into draw groups by Z buffer usage
                DrawGroupMTLRef drawGroup;
                bool dgZBufferRead = false, dgZBufferWrite = false;
                for (const auto &draw : targetContainer->drawables) {
                    DrawableMTL *drawMTL = dynamic_cast<DrawableMTL *>(draw.get());
                    if (!drawMTL) {
                        wkLogLevel(Error, "SceneRendererMTL: Invalid drawable.  Skipping.");
                        continue;
                    }

                    // Sort out what the zbuffer should be
                    bool zBufferWrite;// = (zBufferMode == zBufferOn);
                    bool zBufferRead;// = (zBufferMode == zBufferOn);
                    if (renderTarget->getTex() != nil) {
                        // Off screen render targets don't like z buffering
                        zBufferRead = false;
                        zBufferWrite = false;
                    } else {
                        // The drawable itself gets a say
                        zBufferRead = drawMTL->getRequestZBuffer();
                        zBufferWrite = drawMTL->getWriteZbuffer();
                    }

                    // If this isn't compatible with the draw group, create a new one
                    if (!drawGroup || zBufferRead != dgZBufferRead || zBufferWrite != dgZBufferWrite) {
                        // It's not, so we need to make a new draw group
                        drawGroup = std::make_shared<DrawGroupMTL>();

                        // Depth stencil, which goes in the command encoder later
                        MTLDepthStencilDescriptor *depthDesc = [[MTLDepthStencilDescriptor alloc] init];
                        if (zBufferRead)
                            depthDesc.depthCompareFunction = MTLCompareFunctionLess;
                        else
                            depthDesc.depthCompareFunction = MTLCompareFunctionAlways;
                        depthDesc.depthWriteEnabled = zBufferWrite;
                        
                        drawGroup->depthStencil = [setupInfo.mtlDevice newDepthStencilStateWithDescriptor:depthDesc];
                        
                        targetContainerMTL->drawGroups.push_back(drawGroup);
                        
                        dgZBufferRead = zBufferRead;
                        dgZBufferWrite = zBufferWrite;
                    }
                    drawGroup->drawables.push_back(draw);
                }

                // Command buffer description should be the same
                MTLIndirectCommandBufferDescriptor *cmdBuffDesc = [[MTLIndirectCommandBufferDescriptor alloc] init];
                cmdBuffDesc.commandTypes = MTLIndirectCommandTypeDraw | MTLIndirectCommandTypeDrawIndexed;
                cmdBuffDesc.inheritBuffers = false;
                if (@available(iOS 13.0, *)) {
                    cmdBuffDesc.inheritPipelineState = false;
                }
                // TODO: Should query the drawables to get this maximum number
                cmdBuffDesc.maxVertexBufferBindCount = WhirlyKitShader::WKSVertMaxBuffer;
                cmdBuffDesc.maxFragmentBufferBindCount = WhirlyKitShader::WKSFragMaxBuffer;

                // Build up indirect buffers for each draw group
                for (const auto &drawGroup : targetContainerMTL->drawGroups) {
                    int curCommand = 0;
                    drawGroup->numCommands = drawGroup->drawables.size();
                    drawGroup->indCmdBuff = [setupInfo.mtlDevice newIndirectCommandBufferWithDescriptor:cmdBuffDesc maxCommandCount:drawGroup->numCommands options:0];
                    if (!drawGroup->indCmdBuff) {
                        wkLogLevel(Error, "SceneRendererMTL: Failed to allocate indirect command buffer.  Skipping.");
                        continue;
                    }

                    // Just run the calculation portion
                    if (workGroup->groupType == WorkGroup::Calculation) {
                        // Work through the drawables
                        for (const auto &draw : targetContainer->drawables) {
                            DrawableMTL *drawMTL = dynamic_cast<DrawableMTL *>(draw.get());
                            if (!drawMTL) {
                                wkLogLevel(Error, "SceneRendererMTL: Invalid drawable.  Skipping.");
                                continue;
                            }
                            SimpleIdentity calcProgID = drawMTL->getCalculationProgram();
                            
                            // Figure out the program to use for drawing
                            if (calcProgID == EmptyIdentity)
                                continue;
                            ProgramMTL *calcProgram = (ProgramMTL *)scene->getProgram(calcProgID);
                            if (!calcProgram) {
                                wkLogLevel(Error, "SceneRendererMTL: Invalid calculation program for drawable.  Skipping.");
                                continue;
                            }
                            
                            id<MTLIndirectRenderCommand> cmdEncode = [drawGroup->indCmdBuff indirectRenderCommandAtIndex:curCommand++];
                            drawMTL->encodeIndirectCalculate(cmdEncode,this,scene,renderTarget.get());
                            drawMTL->enumerateResources(baseFrameInfo, drawGroup->resources);
                        }
                    }
                    else
                    {
                        // Work through the drawables
                        for (const auto &draw : drawGroup->drawables)
                        {
                            DrawableMTL *drawMTL = dynamic_cast<DrawableMTL *>(draw.get());
                            if (!drawMTL) {
                                wkLogLevel(Error, "SceneRendererMTL: Invalid drawable");
                                continue;
                            }

                            id<MTLIndirectRenderCommand> cmdEncode = [drawGroup->indCmdBuff indirectRenderCommandAtIndex:curCommand++];

                            auto &frameInfo = *baseFrameInfo;//for (auto &frameInfo : offFrameInfos)
                            {
                                drawMTL->encodeIndirect(cmdEncode,this,scene,renderTarget.get());
                                drawMTL->enumerateResources(&frameInfo, drawGroup->resources);
                            }
                        }
                    }
                }
                
                targetContainer->modified = false;
            }
        }
    }
}

void SceneRendererMTL::render(TimeInterval duration,
                              MTLRenderPassDescriptor *renderPassDesc,
                              id<SceneRendererMTLDrawableGetter> drawGetter)
{
    if (!scene)
        return;
    SceneMTL *sceneMTL = (SceneMTL *)scene;
    
    frameCount++;
    
    const TimeInterval now = scene->getCurrentTime();
    
    teardownInfo = nullptr;

    if (framebufferWidth <= 0 || framebufferHeight <= 0)
    {
        // Process the scene even if the window isn't up
        processScene(now);
        return;
    }
    
    lastDraw = now;
    
    if (perfInterval > 0)
        perfTimer.startTiming("Render Frame");
    
    if (perfInterval > 0)
        perfTimer.startTiming("Render Setup");

    // See if we're dealing with a globe or map view
    Maply::MapView *mapView = dynamic_cast<Maply::MapView *>(theView);
    const float overlapMarginX = mapView ? scene->getOverlapMargin() : 0.0;

    if (!theView) {
        return;
    }

    const Point2f frameSize(framebufferWidth,framebufferHeight);
    const Point2d screenSize = theView->screenSizeInDisplayCoords(frameSize);

    // Get the model and view matrices
    const Matrix4d modelTrans4d = theView->calcModelMatrix();
    const Matrix4d viewTrans4d = theView->calcViewMatrix();
    const Matrix4f modelTrans = Matrix4dToMatrix4f(modelTrans4d);
    const Matrix4f viewTrans = Matrix4dToMatrix4f(viewTrans4d);
    
    // Set up a projection matrix
    const Matrix4d projMat4d = theView->calcProjectionMatrix(frameSize,0.0);

    const Matrix4d modelAndViewMat4d = viewTrans4d * modelTrans4d;
    const Matrix4d pvMat4d = projMat4d * viewTrans4d;
    const Matrix4d modelAndViewNormalMat4d = modelAndViewMat4d.inverse().transpose();
    const Matrix4d mvpMat4d = projMat4d * modelAndViewMat4d;

    const Matrix4f projMat = Matrix4dToMatrix4f(projMat4d);
    const Matrix4f modelAndViewMat = Matrix4dToMatrix4f(modelAndViewMat4d);
    const Matrix4f mvpMat = Matrix4dToMatrix4f(mvpMat4d);
    const Matrix4f mvpNormalMat4f = Matrix4dToMatrix4f(mvpMat4d.inverse().transpose());
    const Matrix4f modelAndViewNormalMat = Matrix4dToMatrix4f(modelAndViewNormalMat4d);

    if (perfInterval > 0)
        perfTimer.stopTiming("Render Setup");

    RenderTargetMTL *defaultTarget = (RenderTargetMTL *)renderTargets.back().get();
    if (renderPassDesc)
        defaultTarget->setRenderPassDesc(renderPassDesc);
    const auto &clearColor = defaultTarget->clearColor;
    renderPassDesc.colorAttachments[0].clearColor = MTLClearColorMake(clearColor[0],clearColor[1],clearColor[2],clearColor[3]);

    // Send the command buffer and encoders
    id<MTLDevice> mtlDevice = setupInfo.mtlDevice;
    id<MTLCommandQueue> cmdQueue = [mtlDevice newCommandQueue];

    RendererFrameInfoMTL baseFrameInfo;
    baseFrameInfo.sceneRenderer = this;
    baseFrameInfo.theView = theView;
    baseFrameInfo.viewTrans = viewTrans;
    baseFrameInfo.viewTrans4d = viewTrans4d;
    baseFrameInfo.modelTrans = modelTrans;
    baseFrameInfo.modelTrans4d = modelTrans4d;
    baseFrameInfo.scene = scene;
    baseFrameInfo.frameLen = duration;
    baseFrameInfo.currentTime = now;
    baseFrameInfo.projMat = projMat;
    baseFrameInfo.projMat4d = projMat4d;
    baseFrameInfo.mvpMat = mvpMat;
    baseFrameInfo.mvpMat4d = mvpMat4d;
    baseFrameInfo.mvpInvMat = mvpMat.inverse();
    baseFrameInfo.mvpNormalMat = mvpNormalMat4f;
    baseFrameInfo.viewModelNormalMat = modelAndViewNormalMat;
    baseFrameInfo.viewAndModelMat = modelAndViewMat;
    baseFrameInfo.viewAndModelMat4d = modelAndViewMat4d;
    baseFrameInfo.pvMat = Matrix4dToMatrix4f(pvMat4d);
    baseFrameInfo.pvMat4d = pvMat4d;
    baseFrameInfo.screenSizeInDisplayCoords = screenSize;
    baseFrameInfo.lights = &lights;
    baseFrameInfo.renderTarget = nullptr;

    theView->getOffsetMatrices(baseFrameInfo.offsetMatrices, frameSize, overlapMarginX);

    // We need a reverse of the eye vector in model space
    // We'll use this to determine what's pointed away
    {
        const Eigen::Matrix4f modelTransInv = modelTrans.inverse();
        const Vector4f eyeVec4 = modelTransInv * Vector4f(0,0,1,0);
        const Vector3f eyeVec3(eyeVec4.x(),eyeVec4.y(),eyeVec4.z());
        baseFrameInfo.eyeVec = eyeVec3;
        const Vector4f fullEyeVec4 = modelAndViewMat.inverse() * Vector4f(0,0,1,0);
        const Vector3f fullEyeVec3(fullEyeVec4.x(),fullEyeVec4.y(),fullEyeVec4.z());
        baseFrameInfo.fullEyeVec = -fullEyeVec3;
        const Matrix4d modelTransInv4d = modelTrans4d.inverse();
        const Vector4d eyeVec4d = modelTransInv4d * Vector4d(0,0,1,0.0);
        baseFrameInfo.heightAboveSurface = theView->heightAboveSurface();

        if (scene->getCoordAdapter()->isFlat())
        {
            const Vector4d eyePos4d = modelTransInv4d * Vector4d(0.0,0.0,0.0,1.0);
            baseFrameInfo.eyePos = Vector3d(eyePos4d.x(),eyePos4d.y(),eyePos4d.z()) / eyePos4d.w();
        }
        else
        {
            baseFrameInfo.eyePos = Vector3d(eyeVec4d.x(),eyeVec4d.y(),eyeVec4d.z()) * (1.0+baseFrameInfo.heightAboveSurface);
        }
    }

    if (perfInterval > 0)
        perfTimer.startTiming("Scene preprocessing");
    
    RenderTeardownInfoMTLRef frameTeardownInfo = std::make_shared<RenderTeardownInfoMTL>();
    teardownInfo = frameTeardownInfo;
    
    // Run the preprocess for the changes.  These modify things the active models need.
    int numPreProcessChanges = preProcessScene(now);;
    
    if (perfInterval > 0)
        perfTimer.addCount("Preprocess Changes", numPreProcessChanges);
    
    if (perfInterval > 0)
        perfTimer.stopTiming("Scene preprocessing");
    
    if (perfInterval > 0)
        perfTimer.startTiming("Active Model Runs");
    
    // Let the active models to their thing
    // That thing had better not take too long
    auto &activeModels = scene->getActiveModels();
    for (auto &activeModel : activeModels)
    {
        activeModel->updateForFrame(&baseFrameInfo);
    }

    if (perfInterval > 0)
        perfTimer.addCount("Active Models", (int)activeModels.size());
    
    if (perfInterval > 0)
        perfTimer.stopTiming("Active Model Runs");
    
    if (perfInterval > 0)
        perfTimer.addCount("Scene changes", scene->getNumChangeRequests());

    // Work through the available offset matrices (only 1 if we're not wrapping)
    const std::vector<Matrix4d> &offsetMats = baseFrameInfo.offsetMatrices;

    offFrameInfos.clear();

    for (const auto &offMat : offsetMats)
    {
        offFrameInfos.emplace_back(baseFrameInfo);
        RendererFrameInfoMTL &offFrameInfo = offFrameInfos.back();

        const Eigen::Matrix4d projMat4d = theView->calcProjectionMatrix(frameSize,0.0);

        // Tweak with the appropriate offset matrix
        offFrameInfo.viewAndModelMat4d = viewTrans4d * offMat * modelTrans4d;
        offFrameInfo.viewAndModelMat = offFrameInfo.viewAndModelMat4d.cast<float>();
        offFrameInfo.viewModelNormalMat = offFrameInfo.viewAndModelMat4d.inverse().transpose().cast<float>();
        offFrameInfo.mvpMat4d = projMat4d * offFrameInfo.viewAndModelMat4d;
        offFrameInfo.mvpMat = offFrameInfo.mvpMat4d.cast<float>();
        offFrameInfo.mvpInvMat = offFrameInfo.mvpMat4d.inverse().cast<float>();
        offFrameInfo.mvpNormalMat = offFrameInfo.mvpMat4d.inverse().transpose().cast<float>();
        offFrameInfo.pvMat4d = projMat4d * viewTrans4d * offMat;
        offFrameInfo.pvMat = offFrameInfo.pvMat4d.cast<float>();
    }

    if (perfInterval > 0)
        perfTimer.startTiming("Scene processing");
    
    // Merge any outstanding changes into the scenegraph
    processScene(now);
    
    // Update our work groups accordingly
    // (uses offFrameInfos)
    updateWorkGroups(&baseFrameInfo);
    
    if (perfInterval > 0)
        perfTimer.stopTiming("Scene processing");

    // Keeps us from stomping on the last frame's uniforms
    if (renderEvent == nil && drawGetter)
        renderEvent = [mtlDevice newEvent];

    id <MTLEvent> offsetRenderedEvent = [mtlDevice newEvent];
    long long offsetEventValue = 1;

    // Workgroups force us to draw things in order
    bool firstFrame = true;
    for (int groupIndex = 0; groupIndex < workGroups.size(); ++groupIndex)
    {
        auto &workGroup = workGroups[groupIndex];
        
        if (perfInterval > 0)
            perfTimer.startTiming("Work Group: " + workGroup->name);

        for (int targetIndex = 0; targetIndex < workGroup->renderTargetContainers.size(); ++targetIndex)
        {
            auto &targetContainer = workGroup->renderTargetContainers[targetIndex];

            // We'll skip empty render targets, except for the default one which we need at least to clear
            // Otherwise we get stuck on the last render, rather than a blank screen
            if (targetContainer->drawables.empty() &&
                !(targetContainer && targetContainer->renderTarget))
            {
                continue;
            }

            const auto targetContainerMTL = dynamic_cast<RenderTargetContainerMTL*>(targetContainer.get());

            // Need some sort of render target even if we're not really rendering
            const auto renderTarget = dynamic_cast<RenderTargetMTL*>(
                (targetContainer->renderTarget ? targetContainer->renderTarget : renderTargets.back()).get());

            // Render pass descriptor might change from frame to frame if we're clearing sporadically
            renderTarget->makeRenderPassDesc();

            baseFrameInfo.renderTarget = renderTarget;

            // Each render target needs its own buffer and command queue
            if (lastCmdBuff)
            {
                // Otherwise we'll commit twice
                if (drawGetter)
                {
                    [lastCmdBuff commit];
                }
                lastCmdBuff = nil;
            }
            const id<MTLCommandBuffer> cmdBuff = [cmdQueue commandBuffer];

            // Keeps us from stomping on the last frame's uniforms
            if (lastRenderNo > 0 && drawGetter)
            {
                [cmdBuff encodeWaitForEvent:renderEvent value:lastRenderNo];
            }

            // Resources used by this container
            ResourceRefsMTL resources;

            for (int offIndex = 0; offIndex < offFrameInfos.size(); ++offIndex)
            {
                if (!firstFrame)
                {
                    // Wait for the previous offset to finish
                    [cmdBuff encodeWaitForEvent:offsetRenderedEvent value:offsetEventValue];
                }

                auto &frameInfo = offFrameInfos[offIndex];
                frameInfo.renderTarget = renderTarget;

                // Ask all the drawables to set themselves up.  Mostly memory stuff.
                id<MTLFence> preProcessFence = [mtlDevice newFence];
                id<MTLBlitCommandEncoder> bltEncode = [cmdBuff blitCommandEncoder];

                if (indirectRender) {
                    // Run pre-process on the draw groups
                    for (const auto &drawGroup : targetContainerMTL->drawGroups) {
                        if (drawGroup->numCommands > 0) {
                            bool resourcesChanged = false;
                            for (auto &draw : drawGroup->drawables) {
                                DrawableMTL *drawMTL = dynamic_cast<DrawableMTL *>(draw.get());
                                if (!drawMTL) {
                                    wkLogLevel(Error, "SceneRendererMTL: Invalid drawable.  Skipping.");
                                    continue;
                                }
                                drawMTL->runTweakers(&frameInfo);
                                if (drawMTL->preProcess(this, cmdBuff, bltEncode, sceneMTL))
                                    resourcesChanged = true;
                            }
                            // At least one of the drawables is pointing at different resources, so we need to redo this
                            if (resourcesChanged) {
                                drawGroup->resources.clear();
                                for (const auto &draw : drawGroup->drawables) {
                                    if (const auto drawMTL = dynamic_cast<DrawableMTL *>(draw.get())) {
                                        drawMTL->enumerateResources(&frameInfo, drawGroup->resources);
                                    }
                                }
                            }
                            resources.addResources(drawGroup->resources);
                        }
                    }
                }
                else
                {
                    // Run pre-process ahead of time
                    for (const auto &draw : targetContainer->drawables)
                    {
                        if (const auto drawMTL = dynamic_cast<DrawableMTL *>(draw.get()))
                        {
                            drawMTL->runTweakers(&frameInfo);
                            drawMTL->preProcess(this, cmdBuff, bltEncode, sceneMTL);
                            drawMTL->enumerateResources(&frameInfo, resources);
                        }
                    }
                }

                // TODO: Just set these up once and copy it into position
                setupLightBuffer(sceneMTL,&frameInfo,bltEncode);
                setupUniformBuffer(&frameInfo,bltEncode,scene->getCoordAdapter());
                [bltEncode updateFence:preProcessFence];
                [bltEncode endEncoding];
                
                // If we're forcing a mipmap calculation, then we're just going to use this render target once
                // If not, then we run some program over it multiple times
                // TODO: Make the reduce operation more explicit
                const int numLevels = (renderTarget->mipmapType == RenderTargetMipmapNone) ? renderTarget->numLevels() : 1;

                for (unsigned int level=0;level<numLevels;level++)
                {
                    // TODO: Pass the level into the draw call
                    //       Also do something about the offset matrices
                    // Set up the encoder
                    id<MTLRenderCommandEncoder> cmdEncode = nil;
                    if (renderTarget->getTex() == nil)
                    {
                        // This happens if the dev wants an instantaneous render
                        if (!renderPassDesc)
                        {
                            renderPassDesc = renderTarget->getRenderPassDesc(level);
                        }

                        frameInfo.renderPassDesc = renderPassDesc;
                    }
                    else
                    {
                        frameInfo.renderPassDesc = renderTarget->getRenderPassDesc(level);
                    }

                    const bool lastFrame = (offIndex == offFrameInfos.size() - 1 && level == numLevels - 1 && groupIndex == workGroups.size() - 1);

                    frameInfo.renderPassDesc.colorAttachments[0].loadAction = firstFrame ? MTLLoadActionClear : MTLLoadActionLoad;
                    frameInfo.renderPassDesc.colorAttachments[0].storeAction =  lastFrame ? MTLStoreActionDontCare : MTLStoreActionStore;
                    frameInfo.renderPassDesc.depthAttachment.loadAction = firstFrame ? MTLLoadActionClear : MTLLoadActionLoad;
                    frameInfo.renderPassDesc.depthAttachment.storeAction = lastFrame ? MTLStoreActionDontCare : MTLStoreActionStore;
                    frameInfo.renderPassDesc.depthAttachment.clearDepth = 1.0f;
                    frameInfo.renderPassDesc.stencilAttachment.loadAction = firstFrame ? MTLLoadActionClear : MTLLoadActionLoad;
                    frameInfo.renderPassDesc.stencilAttachment.storeAction =  lastFrame ? MTLStoreActionDontCare : MTLStoreActionStore;

                    cmdEncode = [cmdBuff renderCommandEncoderWithDescriptor:frameInfo.renderPassDesc];

                    resources.use(cmdEncode);

                    if (indirectRender) {
                        if (@available(iOS 12.0, *)) {
                            [cmdEncode setCullMode:MTLCullModeFront];
                            for (const auto &drawGroup : targetContainerMTL->drawGroups) {
                                if (drawGroup->numCommands > 0) {
                                    [cmdEncode setDepthStencilState:drawGroup->depthStencil];
                                    [cmdEncode executeCommandsInBuffer:drawGroup->indCmdBuff withRange:NSMakeRange(0,drawGroup->numCommands)];
                                }
                            }
                        }
                    } else {
                        // Just run the calculation portion
                        if (workGroup->groupType == WorkGroup::Calculation) {
                            // Work through the drawables
                            for (const auto &draw : targetContainer->drawables) {
                                DrawableMTL *drawMTL = dynamic_cast<DrawableMTL *>(draw.get());
                                if (!drawMTL) {
                                    wkLogLevel(Error, "SceneRendererMTL: Invalid drawable.  Skipping.");
                                    continue;
                                }
                                const SimpleIdentity calcProgID = drawMTL->getCalculationProgram();
                                
                                // Figure out the program to use for drawing
                                if (calcProgID == EmptyIdentity)
                                    continue;

                                ProgramMTL *calcProgram = (ProgramMTL *)scene->getProgram(calcProgID);
                                if (!calcProgram) {
                                    wkLogLevel(Error, "SceneRendererMTL: Invalid calculation program for drawable.  Skipping.");
                                    continue;
                                }
                                frameInfo.program = calcProgram;
                                
                                // Tweakers probably not necessary, but who knows
                                draw->runTweakers(&frameInfo);
                                
                                // Run the calculation phase
                                drawMTL->encodeDirectCalculate(&frameInfo,cmdEncode,scene);
                            }
                        }
                        else
                        {
                            // Keep track of state changes for z buffer state
                            bool firstDepthState = true;
                            bool zBufferWrite = (zBufferMode == zBufferOn);
                            bool zBufferRead = (zBufferMode == zBufferOn);

                            bool lastZBufferWrite = zBufferWrite;
                            bool lastZBufferRead = zBufferRead;

                            // Backface culling on by default
                            // Note: Would like to not set this every time
                            [cmdEncode setCullMode:MTLCullModeFront];

                            // Work through the drawables
                            for (const auto &draw : targetContainer->drawables)
                            {
                                const auto drawMTL = dynamic_cast<DrawableMTL*>(draw.get());
                                if (!drawMTL)
                                {
                                    wkLogLevel(Error, "SceneRendererMTL: Invalid drawable.  Skipping.");
                                    continue;
                                }

                                // Figure out the program to use for drawing
                                ProgramMTL *program = (ProgramMTL *)scene->getProgram(drawMTL->getProgram());
                                if (!program) {
                                    program = (ProgramMTL *)scene->getProgram(drawMTL->getCalculationProgram());
                                    if (!program) {
                                        wkLogLevel(Error, "SceneRendererMTL: Drawable without Program");
                                        continue;
                                    }
                                }

                                // For a reduce operation, we want to draw into the first level of the render
                                //  target texture and then run the reduce over the rest of those levels
                                if (level > 0 && program->getReduceMode() == Program::None)
                                    continue;

                                // For this mode we turn the z buffer off until we get a request to turn it on
                                zBufferRead = drawMTL->getRequestZBuffer();
                                
                                // If we're drawing lines or points we don't want to update the z buffer
                                zBufferWrite = drawMTL->getWriteZbuffer();
                                
                                // Off screen render targets don't like z buffering
                                if (renderTarget->getTex() != nil) {
                                    zBufferRead = false;
                                    zBufferWrite = false;
                                }

                                // TODO: Optimize this a bit
                                if (firstDepthState ||
                                    (zBufferRead != lastZBufferRead) ||
                                    (zBufferWrite != lastZBufferWrite))
                                {
                                    MTLDepthStencilDescriptor *depthDesc = [[MTLDepthStencilDescriptor alloc] init];
                                    depthDesc.depthCompareFunction = zBufferRead ? MTLCompareFunctionLess : MTLCompareFunctionAlways;
                                    depthDesc.depthWriteEnabled = zBufferWrite;

                                    lastZBufferRead = zBufferRead;
                                    lastZBufferWrite = zBufferWrite;
                                    
                                    id<MTLDepthStencilState> depthStencil = [mtlDevice newDepthStencilStateWithDescriptor:depthDesc];
                                    
                                    [cmdEncode setDepthStencilState:depthStencil];
                                    firstDepthState = false;
                                }

                                frameInfo.program = program;

                                // "Draw" using the given program
                                drawMTL->encodeDirect(&frameInfo,cmdEncode,scene);
                            }
                        }
                    }

                    [cmdEncode endEncoding];
                    cmdEncode = nil;

                    // Signal completion
                    offsetEventValue += 1;
                    [cmdBuff encodeSignalEvent:offsetRenderedEvent value:offsetEventValue];

                    firstFrame = false;
                }
            }

            // Some render targets like to do extra work on their images
            renderTarget->addPostProcessing(mtlDevice,cmdBuff);

            // Main screen has to be committed
            if (drawGetter != nil && workGroup->groupType == WorkGroup::ScreenRender) {
                id<CAMetalDrawable> drawable = [drawGetter getDrawable];
                [cmdBuff presentDrawable:drawable];
            }

            // Capture shutdown signal in case `this` is destroyed before the blocks below execute.
            // This isn't 100% because we could still be destroyed while the blocks are executing,
            // unless we can be guaranteed that we're always destroyed on the main queue?
            // We might need `std::enable_shared_from_this` here so that we can keep `this` alive
            // within the blocks we create here.
            const auto shuttingDown = this->_isShuttingDown;

            // This particular target may want a snapshot
            [cmdBuff addCompletedHandler:^(id<MTLCommandBuffer> _Nonnull) {
                if (*shuttingDown)
                    return;

                // TODO: Sort these into the render targets
                dispatch_async(dispatch_get_main_queue(), ^{
                    if (*shuttingDown)
                        return;

                    // Look for the snapshot delegate that wants this render target
                    for (auto snapshotDelegate : snapshotDelegates) {
                        if (*shuttingDown) {
                            break;
                        }
                        
                        if (![snapshotDelegate needSnapshot:now])
                            continue;
                        
                        if (renderTarget->getId() != [snapshotDelegate renderTargetID]) {
                            continue;
                        }
                        
                        [snapshotDelegate snapshotData:nil];
                    }
                    
//                    targetContainerMTL->lastRenderFence = nil;
                    
                    // We can do the free-ing on a low priority queue
                    // But it has to be a single queue, otherwise we'll end up deleting things at the same time.  Oops.
                    dispatch_async(releaseQueue, ^{
                        frameTeardownInfo->clear();
                    });
                });
            }];
            lastCmdBuff = cmdBuff;

            // This happens for offline rendering and we want to wait until the render finishes to return it
            if (!drawGetter) {
                [cmdBuff commit];
                [cmdBuff waitUntilCompleted];
                lastCmdBuff = nil;
            }
        }
                
        if (perfInterval > 0)
            perfTimer.stopTiming("Work Group: " + workGroup->name);
    }
    
    // Notify anyone waiting that this frame is complete
    if (lastCmdBuff) {
        if (drawGetter) {
            [lastCmdBuff encodeSignalEvent:renderEvent value:lastRenderNo+1];
            [lastCmdBuff commit];
        }
        lastCmdBuff = nil;
    }
    lastRenderNo++;

    offFrameInfos.clear();

    if (perfInterval > 0)
        perfTimer.stopTiming("Render Frame");
    
    // Update the frames per sec
    if (perfInterval > 0 && frameCount > perfInterval)
    {
        const TimeInterval now = TimeGetCurrent();
        const TimeInterval howLong =  now - frameCountStart;
        framesPerSec = (howLong > 0) ? frameCount / howLong : 0.;
        frameCountStart = now;
        frameCount = 0;
        
        wkLogLevel(Verbose,"---Rendering Performance---");
        wkLogLevel(Verbose," Frames per sec = %.2f",framesPerSec);
        perfTimer.log();
        perfTimer.clear();
    }
    
    // Mark any programs that changed as now caught up
    scene->markProgramsUnchanged();
    
    // No teardown info available between frames
    if (teardownInfo) {
        teardownInfo.reset();
    }
    
    setupInfo.heapManage.updateHeaps();
}

void SceneRendererMTL::shutdown()
{
    *_isShuttingDown = true;
    
    if (lastCmdBuff)
        [lastCmdBuff waitUntilCompleted];
    lastCmdBuff = nil;
    
    snapshotDelegates.clear();
    
    auto drawables = scene->getDrawables();
    for (auto draw: drawables)
        draw->teardownForRenderer(nil, NULL, NULL);

    SceneRenderer::shutdown();
}

RenderTargetMTLRef SceneRendererMTL::getRenderTarget(SimpleIdentity renderTargetID)
{
    if (renderTargetID == EmptyIdentity) {
        return std::dynamic_pointer_cast<RenderTargetMTL>(renderTargets.back());
    } else {
        for (auto target : renderTargets) {
            if (target->getId() == renderTargetID) {
                return std::dynamic_pointer_cast<RenderTargetMTL>(target);
            }
        }
    }
    return RenderTargetMTLRef();
}

RawDataRef SceneRendererMTL::getSnapshot(SimpleIdentity renderTargetID)
{
    const auto renderTarget = getRenderTarget(renderTargetID);
    return renderTarget ? renderTarget->snapshot() : nil;
}

RawDataRef SceneRendererMTL::getSnapshotAt(SimpleIdentity renderTargetID,int x,int y)
{
    const auto renderTarget = getRenderTarget(renderTargetID);
    return renderTarget ? renderTarget->snapshot(x, y, 1, 1) : nil;
}

RawDataRef SceneRendererMTL::getSnapshotMinMax(SimpleIdentity renderTargetID)
{
    const auto renderTarget = getRenderTarget(renderTargetID);
    return renderTarget ? renderTarget->snapshotMinMax() : nil;
}
    
BasicDrawableBuilderRef SceneRendererMTL::makeBasicDrawableBuilder(const std::string &name) const
{
    return std::make_shared<BasicDrawableBuilderMTL>(name,scene);
}

BasicDrawableInstanceBuilderRef SceneRendererMTL::makeBasicDrawableInstanceBuilder(const std::string &name) const
{
    return std::make_shared<BasicDrawableInstanceBuilderMTL>(name,scene);
}

BillboardDrawableBuilderRef SceneRendererMTL::makeBillboardDrawableBuilder(const std::string &name) const
{
    return std::make_shared<BillboardDrawableBuilderMTL>(name,scene);
}

ScreenSpaceDrawableBuilderRef SceneRendererMTL::makeScreenSpaceDrawableBuilder(const std::string &name) const
{
    return std::make_shared<ScreenSpaceDrawableBuilderMTL>(name,scene);
}

ParticleSystemDrawableBuilderRef  SceneRendererMTL::makeParticleSystemDrawableBuilder(const std::string &name) const
{
    return nullptr;
}

WideVectorDrawableBuilderRef SceneRendererMTL::makeWideVectorDrawableBuilder(const std::string &name) const
{
    return std::make_shared<WideVectorDrawableBuilderMTL>(name,this,scene);
}

RenderTargetRef SceneRendererMTL::makeRenderTarget() const
{
    return std::make_shared<RenderTargetMTL>();
}

DynamicTextureRef SceneRendererMTL::makeDynamicTexture(const std::string &name) const
{
    return std::make_shared<DynamicTextureMTL>(name);
}

    
}
