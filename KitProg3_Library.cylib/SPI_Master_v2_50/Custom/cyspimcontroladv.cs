/*******************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Diagnostics;
using System.Data;
using System.Text;
using System.Windows.Forms;
using CyDesigner.Extensions.Common;
using CyDesigner.Extensions.Gde;

namespace SPI_Master_v2_50
{
    public partial class CySPIMControlAdv : UserControl, ICyParamEditingControl
    {
        private CySPIMParameters m_params;

        #region Constructor(s)
        public CySPIMControlAdv(CySPIMParameters inst)
        {
            InitializeComponent();

            inst.m_advTab = this;
            this.Dock = DockStyle.Fill;
            this.AutoScrollMinSize = new Size(this.Width - 20, this.Height);
            m_params = inst;
            numRXBufferSize.TextChanged += new EventHandler(numRXBufferSize_TextChanged);
            numTXBufferSize.TextChanged += new EventHandler(numTXBufferSize_TextChanged);
            // NumericsUpDown Settings
            numRXBufferSize.Minimum = 0;
            numRXBufferSize.Maximum = uint.MaxValue;
            numTXBufferSize.Minimum = 0;
            numTXBufferSize.Maximum = uint.MaxValue;
        }
        #endregion

        #region ICyParamEditingControl Members
        public Control DisplayControl
        {
            get { return this; }
        }

        public IEnumerable<CyCustErr> GetErrors()
        {
            foreach (string paramName in m_params.m_inst.GetParamNames())
            {
                CyCompDevParam param = m_params.m_inst.GetCommittedParam(paramName);
                if (param.TabName.Equals(CyCustomizer.ADVANCED_TABNAME))
                {
                    if (param.ErrorCount > 0)
                    {
                        foreach (string errMsg in param.Errors)
                        {
                            yield return new CyCustErr(errMsg);
                        }
                    }
                }
            }
        }
        #endregion

        #region Assigning parameters values to controls
        public void UpdateUI()
        {
            // ClockInternal
            rbInternalClock.Checked = m_params.ClockInternal;
            rbExternalClock.Checked = !rbInternalClock.Checked;
            m_params.m_basicTab.SetBitRateAvailability();

            // RxBufferSize
            numRXBufferSize.Value = (decimal)m_params.RxBufferSize;

            // TxBufferSize
            numTXBufferSize.Value = (decimal)m_params.TxBufferSize;

            // HighSpeedMode
            chbxEnableHighSpeedMode.Checked = m_params.HighSpeedMode;

            // UseInternalInterrupt
            chbxEnableTXInternalInterrupt.Checked = m_params.UseTxInternalInterrupt;

            // UseInternalInterrupt
            chbxEnableRXInternalInterrupt.Checked = m_params.UseRxInternalInterrupt;

            // InterruptOnTXFull
            chbxIntOnRXFull.Checked = m_params.InterruptOnRXFull;

            // InterruptOnTXNotFull
            chbxIntOnTXNotFull.Checked = m_params.InterruptOnTXNotFull;

            // InterruptOnSPIDone
            chbxIntOnSPIDone.Checked = m_params.InterruptOnSPIDone;

            // InterruptOnSPIDone
            chbxIntOnSPIIdle.Checked = m_params.InterruptOnSPIIdle;

            // InterruptOnRXOverrun
            chbxIntOnRXOverrun.Checked = m_params.InterruptOnRXOverrun;

            // InterruptOnRXEmpty
            chbxIntOnTXEmpty.Checked = m_params.InterruptOnTXEmpty;

            // InterruptOnRXNotEmpty
            chbxIntOnRXNotEmpty.Checked = m_params.InterruptOnRXNotEmpty;

            // InterruptOnByteComplete
            chbxIntOnByteComplete.Checked = m_params.InterruptOnByteComplete;
        }
        #endregion

        #region Assigning controls values to parameters
        private void SetUseTxInternalInterrupt()
        {
            m_params.UseTxInternalInterrupt = chbxEnableTXInternalInterrupt.Checked;
        }

        private void SetUseRxInternalInterrupt()
        {
            m_params.UseRxInternalInterrupt = chbxEnableRXInternalInterrupt.Checked;
        }
        #endregion

        #region Errors handling
        public void ShowError(string paramName, CyCustErr err)
        {
            Control control = null;
            string errMsg = (err.IsOk) ? string.Empty : err.Message;
            switch (paramName)
            {
                case CyParamNames.CLOCK_INTERNAL:
                    control = rbExternalClock;
                    break;
                case CyParamNames.INTERRUPT_ON_BYTE_COMPLETE:
                    control = chbxIntOnByteComplete;
                    break;
                case CyParamNames.INTERRUPT_ON_RX_FULL:
                    control = chbxIntOnRXFull;
                    break;
                case CyParamNames.INTERRUPT_ON_RX_NOT_EMPTY:
                    control = chbxIntOnRXNotEmpty;
                    break;
                case CyParamNames.INTERRUPT_ON_RX_OVERRUN:
                    control = chbxIntOnRXOverrun;
                    break;
                case CyParamNames.INTERRUPT_ON_SPI_DONE:
                    control = chbxIntOnSPIDone;
                    break;
                case CyParamNames.INTERRUPT_ON_SPI_IDLE:
                    control = chbxIntOnSPIIdle;
                    break;
                case CyParamNames.INTERRUPT_ON_TX_EMPTY:
                    control = chbxIntOnTXEmpty;
                    break;
                case CyParamNames.INTERRUPT_ON_TX_NOT_FULL:
                    control = chbxIntOnTXNotFull;
                    break;
                case CyParamNames.RX_BUFFER_SIZE:
                    control = numRXBufferSize;
                    break;
                case CyParamNames.TX_BUFFER_SIZE:
                    control = numTXBufferSize;
                    break;
                case CyParamNames.USE_RX_INTERNAL_INTERRUPT:
                    control = chbxEnableRXInternalInterrupt;
                    break;
                case CyParamNames.USE_TX_INTERNAL_INTERRUPT:
                    control = chbxEnableTXInternalInterrupt;
                    break;
                default:
                    break;
            }
            try
            {
                ep_Errors.SetError(control, errMsg);
            }
            catch (Exception) { }
        }
        #endregion

        #region Event Handlers
        private bool NumUpDownValidated(object sender)
        {
            NumericUpDown numericUpDown = (NumericUpDown)sender;
            string message = "";
            bool result = false;
            int value = 0;
            if (numericUpDown == numRXBufferSize)
            {
                message = String.Format(Properties.Resources.RXBufferSizeEPMsg,
                    CyParamRange.BUFFER_SIZE_MIN, CyParamRange.BUFFER_SIZE_MAX);
            }
            else if (numericUpDown == numTXBufferSize)
            {
                message = String.Format(Properties.Resources.TXBufferSizeEPMsg,
                    CyParamRange.BUFFER_SIZE_MIN, CyParamRange.BUFFER_SIZE_MAX);
            }
            if (int.TryParse(numericUpDown.Text, out value))
            {
                if (value < CyParamRange.BUFFER_SIZE_MIN || value > CyParamRange.BUFFER_SIZE_MAX)
                {
                    ep_Errors.SetError(numericUpDown, message);
                }
                else
                {
                    ep_Errors.SetError(numericUpDown, "");
                    result = true;
                }
            }
            else
            { ep_Errors.SetError(numericUpDown, message); }
            return result;
        }

        private void numRXBufferSize_TextChanged(object sender, EventArgs e)
        {
            if (NumUpDownValidated(numRXBufferSize))
            {
                int rxVal = int.Parse(numRXBufferSize.Text);

                bool on = (rxVal > 4);

                chbxEnableRXInternalInterrupt.Checked = on;
                chbxEnableRXInternalInterrupt.Enabled = !on;

                chbxIntOnRXNotEmpty.Checked = on;
                chbxIntOnRXNotEmpty.Enabled = !on;
            }

            try
            {
                m_params.RxBufferSize = byte.Parse(numRXBufferSize.Text);
            }
            catch (Exception)
            {
                m_params.RxBufferSize = null;
            }
        }

        private void numTXBufferSize_TextChanged(object sender, EventArgs e)
        {
            if (NumUpDownValidated(numTXBufferSize))
            {
                int txVal = int.Parse(numTXBufferSize.Text);

                bool on = (txVal > 4);

                chbxEnableTXInternalInterrupt.Checked = on;
                chbxEnableTXInternalInterrupt.Enabled = !on;

                chbxIntOnTXNotFull.Checked = on;
                chbxIntOnTXNotFull.Enabled = !on;
            }

            try
            {
                m_params.TxBufferSize = byte.Parse(numTXBufferSize.Text);
            }
            catch (Exception)
            {
                m_params.TxBufferSize = null;
            }
        }

        private void rbInternalClock_CheckedChanged(object sender, EventArgs e)
        {
            m_params.ClockInternal = rbInternalClock.Checked;
            m_params.m_basicTab.SetBitRateAvailability();
        }

        private void rbExternalClock_CheckedChanged(object sender, EventArgs e)
        {
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void chbxEnableHighSpeedMode_CheckedChanged(object sender, EventArgs e)
        {
            m_params.HighSpeedMode = chbxEnableHighSpeedMode.Checked;
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void chbxIntOnRXFull_CheckedChanged(object sender, EventArgs e)
        {
            m_params.InterruptOnRXFull = ((CheckBox)sender).Checked;
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void chbxIntOnTXNotFull_CheckedChanged(object sender, EventArgs e)
        {
            m_params.InterruptOnTXNotFull = ((CheckBox)sender).Checked;
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void chbxIntOnSPIDone_CheckedChanged(object sender, EventArgs e)
        {
            m_params.InterruptOnSPIDone = ((CheckBox)sender).Checked;
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void chbxIntOnRXOverrun_CheckedChanged(object sender, EventArgs e)
        {
            m_params.InterruptOnRXOverrun = ((CheckBox)sender).Checked;
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void chbxIntOnTXEmpty_CheckedChanged(object sender, EventArgs e)
        {
            m_params.InterruptOnTXEmpty = ((CheckBox)sender).Checked;
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void chbxIntOnRXNotEmpty_CheckedChanged(object sender, EventArgs e)
        {
            m_params.InterruptOnRXNotEmpty = ((CheckBox)sender).Checked;
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void chbxIntOnByteComplete_CheckedChanged(object sender, EventArgs e)
        {
            m_params.InterruptOnByteComplete = ((CheckBox)sender).Checked;
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void chbxEnableTXInternalInterrupt_CheckedChanged(object sender, EventArgs e)
        {
            SetUseTxInternalInterrupt();
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void chbxEnableRXInternalInterrupt_CheckedChanged(object sender, EventArgs e)
        {
            SetUseRxInternalInterrupt();
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void chbxIntOnSPIIdle_CheckedChanged(object sender, EventArgs e)
        {
            m_params.InterruptOnSPIIdle = ((CheckBox)sender).Checked;
            ep_Errors.SetError((Control)sender, string.Empty);
        }

        private void numUpDown_Validating(object sender, CancelEventArgs e)
        {
            NumUpDownValidated(sender);
        }

        private void CySPIMControlAdv_Load(object sender, EventArgs e)
        {
            // Set UseInternalInterrupt parameter to false to allow customizer ignore it next time 
            if (m_params.UseInternalInterrupt)
            {
                SetUseRxInternalInterrupt();
                SetUseTxInternalInterrupt();
                m_params.UseInternalInterrupt = false;
            }
        }

        private void CySPIMControlAdv_VisibleChanged(object sender, EventArgs e)
        {
            if (this.Visible)
            {
                if (m_params.NumberOfDataBits.Value >= CyParamRange.NUM_BITS_MIN
                    && m_params.NumberOfDataBits.Value <= CyParamRange.NUM_BITS_MAX)
                {
                    lblRXBufferSize.Text = string.Format(Properties.Resources.RxBufferSizeLabelText,
                        m_params.NumberOfDataBits.Value);
                    lblTXBufferSize.Text = string.Format(Properties.Resources.TxBufferSizeLabelText,
                        m_params.NumberOfDataBits.Value);
                }
                else
                {
                    lblRXBufferSize.Text = Properties.Resources.RxBufferSizeLabelUnknownText;
                    lblTXBufferSize.Text = Properties.Resources.TxBufferSizeLabelUnknownText;
                }
            }
        }
        #endregion
    }
}
