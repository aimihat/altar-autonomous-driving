/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/
#pragma once
/*
 * This file depends on Qt which is licensed under LGPLv3.
 * See ADTF_DIR/3rdparty/qt590 and doc/license for detailed information.
 */
#define CID_ADTF_DEMO_MEDIA_DESC_DISPLAY  "AltarViz.filter.user.aadc.cid"
class cQtMediaDescFilter : public adtf::ui::cQtUIDynamicFilter
{
public:
    ADTF_CLASS_ID_NAME(cQtMediaDescFilter, CID_ADTF_DEMO_MEDIA_DESC_DISPLAY, "AltarViz");
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::ant::IQtXSystem));
private:
    QTreeWidget* m_pTree;
    QWidget* m_pWidget;
    adtf::mediadescription::cSampleCodecFactory m_oCoderFactory;
    adtf::streaming::cSingleSampleReader m_oStructReader;
    QString m_strPinName;
    struct tTreeItem
    {
        adtf::util::cString strTypeName;
        tUInt8 ui32PrimitiveType;
        QTreeWidgetItem* pItem;
        tBool bStructuralType;
    };
    std::vector<tTreeItem> m_oItemList;
    std::map<adtf::util::cString, QTreeWidgetItem*> m_mapParents;
    QTreeWidgetItem* m_pFirstDynamicItem;
public:
    cQtMediaDescFilter();
    virtual ~cQtMediaDescFilter();
    tResult RequestPin(const tChar* strName,
                       const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType,
                       adtf::ucom::ant::iobject_ptr<adtf::streaming::ant::IInPin>& pInPin) override;
protected: // Implement cBaseQtFilter
    QWidget* CreateView();
    tVoid    ReleaseView();
    tResult OnIdle();
private:
    tResult ChangeType(const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType);
    void UpdateTree();
private:
    tVoid ClearItemList();
    tVoid SetNoMediaDescriptionAvailable();
    tVoid BuildItemList(const QString& strRootName);
    tVoid AddElement(const ddl::tStructElement* pElem,
                     adtf::util::cString& strDisplayName,
                     QTreeWidgetItem* pParent);
    QTreeWidgetItem* GetParent(const adtf::util::cString& strParent,
                               QTreeWidgetItem* pRoot);
    QTreeWidgetItem* AddParent(const adtf::util::cString& strParent,
                               QTreeWidgetItem* pRoot);
};
