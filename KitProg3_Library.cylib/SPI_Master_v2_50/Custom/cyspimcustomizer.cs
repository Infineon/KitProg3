/*******************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

using System;
using System.Collections.Generic;
using System.Text;
using System.Windows.Forms;
using System.Diagnostics;
using CyDesigner.Extensions.Common;
using CyDesigner.Extensions.Gde;

namespace SPI_Master_v2_50
{    
    [CyCompDevCustomizer()]
    public class CyCustomizer : ICyParamEditHook_v1, ICyDRCProvider_v1
    {
        public const string BASIC_TABNAME = "Basic";
        public const string ADVANCED_TABNAME = "Advanced";

        #region ICyParamEditHook_v1 Members
        public DialogResult EditParams(ICyInstEdit_v1 edit, ICyTerminalQuery_v1 termQuery, ICyExpressMgr_v1 mgr)
        {
            CySPIMParameters parameters = new CySPIMParameters(edit);
            ICyTabbedParamEditor editor = edit.CreateTabbedParamEditor();

            CyParamExprDelegate configureExpressionViewDataChanged =
            delegate(ICyParamEditor custEditor, CyCompDevParam param)
            {
                parameters.LoadParameters(edit);
            };
            editor.AddCustomPage(Properties.Resources.BasicTabTitle, new CySPIMControl(parameters),
                configureExpressionViewDataChanged, BASIC_TABNAME);
            editor.AddCustomPage(Properties.Resources.AdvancedTabTitle, new CySPIMControlAdv(parameters),
                configureExpressionViewDataChanged, ADVANCED_TABNAME);
            editor.AddDefaultPage(Properties.Resources.BuiltInTabTitle, "Built-in");
            parameters.LoadParameters(edit);
            parameters.m_bGlobalEditMode = true;
            return editor.ShowDialog();
        }

        public bool EditParamsOnDrop
        {
            get { return false; }
        }

        public CyCompDevParamEditorMode GetEditorMode()
        {
            return CyCompDevParamEditorMode.COMPLETE;
        }
        #endregion

        #region ICyDRCProvider_v1 Members
        public IEnumerable<CyDRCInfo_v1> GetDRCs(ICyDRCProviderArgs_v1 args)
        {
            CyCustErr err = VerifyNumberOfDatabits(args.InstQueryV1);
            if (err.IsOk == false)
                yield return new CyDRCInfo_v1(CyDRCInfo_v1.CyDRCType_v1.Error, err.Message);
        }

        CyCustErr VerifyNumberOfDatabits(ICyInstQuery_v1 instQuery)
        {
            CyCustErr result = CyCustErr.OK;

            byte value = 0;
            instQuery.GetCommittedParam(CyParamNames.NUMBER_OF_DATA_BITS).TryGetValueAs<byte>(out value);

            if (value < CyParamRange.NUM_BITS_MIN)
            {
                result = new CyCustErr(string.Format(Properties.Resources.DRCNumberOfDataBitsMsg, 
                    CyParamRange.NUM_BITS_MIN, CyParamRange.NUM_BITS_MAX));
            }
            return result;
        }
        #endregion
    }
}
