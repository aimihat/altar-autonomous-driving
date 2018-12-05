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


/*
 * This file depends on Qt which is licensed under LGPLv3.
 * See ADTF_DIR/3rdparty/qt590 and doc/license for detailed information.
 */
#include "stdafx.h"
//*************************************************************************************************
ADTF_PLUGIN("AltarViz", cQtMediaDescFilter)
//*************************************************************************************************
cQtMediaDescFilter::cQtMediaDescFilter():
        m_pTree(nullptr)
{
}
cQtMediaDescFilter::~cQtMediaDescFilter()
{

}
tResult cQtMediaDescFilter::RequestPin(const tChar* strName,
                                       const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType,
                                       adtf::ucom::ant::iobject_ptr<adtf::streaming::ant::IInPin>& pInPin)
{
    adtf::ucom::object_list<adtf::streaming::IPin> lstPins;
    RETURN_IF_FAILED(GetPins(lstPins));
    if (lstPins.GetSize() > 0)
    {
        RETURN_ERROR_DESC(ERR_INVALID_ARG, "Can only connect one dynamic pin");
    }
    m_strPinName = strName;
    RETURN_IF_FAILED(adtf::streaming::make_sample_reader(m_oStructReader, strName, pType));
    m_oStructReader.SetAcceptTypeCallback(std::bind(&cQtMediaDescFilter::ChangeType, this, std::placeholders::_1));
    return adtf::filter::filter_create_pin(*this, m_oStructReader, pInPin);
}
QWidget* cQtMediaDescFilter::CreateView()
{
    // Tree View
    m_pTree = new QTreeWidget(nullptr);
    m_pTree->setFrameShape(QFrame::Box);
    m_pTree->setFrameShadow(QFrame::Plain);
    m_pTree->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    m_pTree->setSelectionMode(QAbstractItemView::SingleSelection);
    m_pTree->setAlternatingRowColors(true);
    QStringList oLabels;
    oLabels << QString("Name") << QString("Value");
    m_pTree->setColumnCount(oLabels.count());
    m_pTree->setColumnWidth(0, 190);
    m_pTree->setColumnWidth(1, 150);
    m_pTree->setHeaderLabels(oLabels);
    m_pTree->setRootIsDecorated(true);
    QHeaderView* pTreeHeader = m_pTree->header();
    pTreeHeader->show();
    // Layout
    QGridLayout* pLayout = new QGridLayout();
    pLayout->setMargin(11);
    pLayout->setSpacing(6);
    pLayout->setColumnStretch(0, 1);
    pLayout->setColumnStretch(1, 1);
    pLayout->setRowStretch(0, 1);
    pLayout->setRowStretch(1, 0);
    pLayout->addWidget(m_pTree, 0, 0, 1, 2);
    m_pWidget = new QWidget();
    m_pWidget->setLayout(pLayout);
    SetNoMediaDescriptionAvailable();
    //On a a qt ui filter the CreateView method HAS TO return a QWidget pointer!
    return m_pWidget;
}
tVoid cQtMediaDescFilter::ReleaseView()
{
    if (m_pTree)
    {
        ClearItemList();
        m_pTree = nullptr;
    }
    delete m_pWidget;
    m_pWidget = nullptr;
}
tVoid cQtMediaDescFilter::AddElement(const ddl::tStructElement* pElem,
                                     adtf::util::cString& strDisplayName,
                                     QTreeWidgetItem* pParent){
    QTreeWidgetItem* pItem = new QTreeWidgetItem();
    pItem->setText(0, QString(strDisplayName));
    tTreeItem oItem;
    oItem.pItem = pItem;
    oItem.ui32PrimitiveType = pElem->eType;
    m_oItemList.push_back(oItem);
    pParent->addChild(pItem);
}
tVoid cQtMediaDescFilter::BuildItemList(const QString& strRootName)
{
    QTreeWidgetItem* pRoot = new QTreeWidgetItem(m_pTree, 1);
    if (!strRootName.isEmpty())
    {
        pRoot->setText(0, strRootName);
    }
    else
    {
        pRoot->setText(0, QString("Media Description"));
    }
    tSize nElements = m_oCoderFactory.GetStaticElementCount();
    for (tSize nIdx = 0;
         nIdx < nElements;
         nIdx++)
    {
        const ddl::tStructElement* pElement = nullptr;
        m_oCoderFactory.GetStaticElement(nIdx, pElement);
        adtf::util::cString strParent;
        adtf::util::cString strDisplayName;
        QTreeWidgetItem* pParent = pRoot;
        if (pElement->strName.RSplitByToken('.', strParent, strDisplayName) != adtf::util::cString::InvalidPos)
        {
            if (strParent.IsNotEmpty())
            {
                pParent = GetParent(strParent, pRoot);
            }
            else
            {
                strDisplayName = pElement->strName;
            }
        }
        else
        {
            strDisplayName = pElement->strName;
        }
        AddElement(pElement, strDisplayName, pParent);
    }
    m_pTree->expandItem(pRoot);
}
QTreeWidgetItem* cQtMediaDescFilter::GetParent(const adtf::util::cString& strParent,
                                               QTreeWidgetItem* pRoot)
{
    auto itItem = m_mapParents.find(strParent);
    if (itItem != m_mapParents.end())
    {
        return itItem->second;
    }
    return AddParent(strParent,
                     pRoot);
}
QTreeWidgetItem* cQtMediaDescFilter::AddParent(const adtf::util::cString& strParent,
                                               QTreeWidgetItem* pRoot)
{
    adtf::util::cStringList lstItems;
    strParent.Split(lstItems, '.');
    adtf::util::cString strCurrentPath;
    adtf::util::cString strLastNamePath;
    tSize nIdx = 0;
    QTreeWidgetItem* pCurrentParent = pRoot;
    for (auto it : lstItems)
    {
        if (nIdx == 0)
        {
            strCurrentPath = it;
        }
        else
        {
            strCurrentPath = "." + it;
        }
        auto itFound = m_mapParents.find(strCurrentPath);
        if (itFound != m_mapParents.end())
        {
            pCurrentParent = itFound->second;
        }
        else
        {
            QTreeWidgetItem* pItem = new QTreeWidgetItem();
            pItem->setText(0, QString(it));
            pCurrentParent->addChild(pItem);
            pCurrentParent = pItem;
            m_mapParents[strCurrentPath] = pItem;
        }
    }
    return pCurrentParent;
}
tVoid cQtMediaDescFilter::ClearItemList()
{
    m_oItemList.clear();
    m_mapParents.clear();
    m_pTree->clear();
}
tVoid cQtMediaDescFilter::SetNoMediaDescriptionAvailable()
{
    m_pTree->addTopLevelItem(new QTreeWidgetItem({"no media description available (yet)"}));
}
tResult cQtMediaDescFilter::OnIdle()
{
    m_pTree->setUpdatesEnabled(false);
    UpdateTree();
    m_pTree->setUpdatesEnabled(true);
    return adtf::ui::cQtUIDynamicFilter::OnIdle();
}
// this is called during the GetLastSample call below, so we are fine to update the qt tree within this method
tResult cQtMediaDescFilter::ChangeType(const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType)
{
    m_oCoderFactory = adtf::mediadescription::cSampleCodecFactory();
    ClearItemList();
    if (pType.Get() &&
        IS_OK(adtf::mediadescription::get_codec_factory_from_stream_type(*pType.Get(), m_oCoderFactory)))
    {
        BuildItemList(m_strPinName);
    }
    else
    {
        SetNoMediaDescriptionAvailable();
    }
    RETURN_NOERROR;
}
void cQtMediaDescFilter::UpdateTree()
{
    adtf::ucom::object_ptr<const adtf::streaming::ISample> pCurrentSample;
    if (IS_OK(m_oStructReader.GetLastSample(pCurrentSample)))
    {
        if (IS_OK(m_oCoderFactory.IsValid()))
        {
            adtf::mediadescription::cStaticSampleDecoder oDecoder = m_oCoderFactory.MakeStaticDecoderFor(pCurrentSample);
            if (IS_OK(oDecoder.IsValid()))
            {
                if (oDecoder.GetElementCount() == m_oItemList.size())
                {
                    for (tSize nIdx = 0;
                         nIdx < m_oItemList.size();
                         ++nIdx)
                    {
                        adtf::util::cVariant oValue;
                        oDecoder.GetElementValue(nIdx, oValue);
                        m_oItemList[nIdx].pItem->setText(1, QString(oValue.AsString()));
                    }
                }
            }
        }
    }
}
