namespace SPI_Master_v2_50
{
    partial class CySPIMControl
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.cbMode = new System.Windows.Forms.ComboBox();
            this.m_lblspimMode = new System.Windows.Forms.Label();
            this.m_lblNumDataBits = new System.Windows.Forms.Label();
            this.pbDrawing = new System.Windows.Forms.PictureBox();
            this.lblShiftDir = new System.Windows.Forms.Label();
            this.cbShiftDir = new System.Windows.Forms.ComboBox();
            this.m_lblBitRate = new System.Windows.Forms.Label();
            this.cbBitRateHertz = new System.Windows.Forms.ComboBox();
            this.ep_Errors = new System.Windows.Forms.ErrorProvider(this.components);
            this.numBitRateHertz = new SPI_Master_v2_50.CyNumericUpDown();
            this.numDataBits = new SPI_Master_v2_50.CyNumericUpDown();
            this.cbDataLines = new System.Windows.Forms.ComboBox();
            this.lblCalculatedBitRate = new System.Windows.Forms.Label();
            this.m_lblDataLines = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.pbDrawing)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ep_Errors)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numBitRateHertz)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numDataBits)).BeginInit();
            this.SuspendLayout();
            // 
            // cbMode
            // 
            this.cbMode.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.cbMode.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cbMode.FormattingEnabled = true;
            this.ep_Errors.SetIconAlignment(this.cbMode, System.Windows.Forms.ErrorIconAlignment.MiddleLeft);
            this.cbMode.Location = new System.Drawing.Point(112, 142);
            this.cbMode.Name = "cbMode";
            this.cbMode.Size = new System.Drawing.Size(316, 21);
            this.cbMode.TabIndex = 0;
            this.cbMode.SelectedIndexChanged += new System.EventHandler(this.cbMode_SelectedIndexChanged);
            // 
            // m_lblspimMode
            // 
            this.m_lblspimMode.AutoSize = true;
            this.m_lblspimMode.Location = new System.Drawing.Point(63, 145);
            this.m_lblspimMode.Name = "m_lblspimMode";
            this.m_lblspimMode.Size = new System.Drawing.Size(37, 13);
            this.m_lblspimMode.TabIndex = 2;
            this.m_lblspimMode.Text = "Mode:";
            this.m_lblspimMode.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // m_lblNumDataBits
            // 
            this.m_lblNumDataBits.AutoSize = true;
            this.m_lblNumDataBits.Location = new System.Drawing.Point(47, 198);
            this.m_lblNumDataBits.Name = "m_lblNumDataBits";
            this.m_lblNumDataBits.Size = new System.Drawing.Size(53, 13);
            this.m_lblNumDataBits.TabIndex = 4;
            this.m_lblNumDataBits.Text = "Data Bits:";
            this.m_lblNumDataBits.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // pbDrawing
            // 
            this.pbDrawing.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.pbDrawing.BackColor = System.Drawing.Color.White;
            this.pbDrawing.Location = new System.Drawing.Point(6, 3);
            this.pbDrawing.Name = "pbDrawing";
            this.pbDrawing.Size = new System.Drawing.Size(422, 133);
            this.pbDrawing.TabIndex = 20;
            this.pbDrawing.TabStop = false;
            // 
            // lblShiftDir
            // 
            this.lblShiftDir.AutoSize = true;
            this.lblShiftDir.Location = new System.Drawing.Point(216, 197);
            this.lblShiftDir.Name = "lblShiftDir";
            this.lblShiftDir.Size = new System.Drawing.Size(76, 13);
            this.lblShiftDir.TabIndex = 27;
            this.lblShiftDir.Text = "Shift Direction:";
            this.lblShiftDir.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // cbShiftDir
            // 
            this.cbShiftDir.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cbShiftDir.FormattingEnabled = true;
            this.ep_Errors.SetIconAlignment(this.cbShiftDir, System.Windows.Forms.ErrorIconAlignment.MiddleLeft);
            this.cbShiftDir.Location = new System.Drawing.Point(304, 193);
            this.cbShiftDir.Name = "cbShiftDir";
            this.cbShiftDir.Size = new System.Drawing.Size(124, 21);
            this.cbShiftDir.TabIndex = 3;
            this.cbShiftDir.SelectedIndexChanged += new System.EventHandler(this.cbShiftDir_SelectedIndexChanged);
            // 
            // m_lblBitRate
            // 
            this.m_lblBitRate.AutoSize = true;
            this.m_lblBitRate.Location = new System.Drawing.Point(52, 222);
            this.m_lblBitRate.Name = "m_lblBitRate";
            this.m_lblBitRate.Size = new System.Drawing.Size(48, 13);
            this.m_lblBitRate.TabIndex = 35;
            this.m_lblBitRate.Text = "Bit Rate:";
            this.m_lblBitRate.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // cbBitRateHertz
            // 
            this.cbBitRateHertz.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cbBitRateHertz.FormattingEnabled = true;
            this.ep_Errors.SetIconAlignment(this.cbBitRateHertz, System.Windows.Forms.ErrorIconAlignment.MiddleLeft);
            this.cbBitRateHertz.Items.AddRange(new object[] {
            "kbps",
            "Mbps"});
            this.cbBitRateHertz.Location = new System.Drawing.Point(218, 218);
            this.cbBitRateHertz.Name = "cbBitRateHertz";
            this.cbBitRateHertz.Size = new System.Drawing.Size(77, 21);
            this.cbBitRateHertz.TabIndex = 5;
            this.cbBitRateHertz.SelectedIndexChanged += new System.EventHandler(this.cbBitRateHertz_SelectedIndexChanged);
            // 
            // ep_Errors
            // 
            this.ep_Errors.ContainerControl = this;
            // 
            // numBitRateHertz
            // 
            this.numBitRateHertz.AllowNegative = false;
            this.numBitRateHertz.DecimalPlaces = 3;
            this.ep_Errors.SetIconAlignment(this.numBitRateHertz, System.Windows.Forms.ErrorIconAlignment.MiddleLeft);
            this.numBitRateHertz.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numBitRateHertz.Location = new System.Drawing.Point(112, 219);
            this.numBitRateHertz.Maximum = new decimal(new int[] {
            30000,
            0,
            0,
            0});
            this.numBitRateHertz.Name = "numBitRateHertz";
            this.numBitRateHertz.Size = new System.Drawing.Size(100, 20);
            this.numBitRateHertz.TabIndex = 4;
            this.numBitRateHertz.Leave += new System.EventHandler(this.numBitRateHertz_Leave);
            // 
            // numDataBits
            // 
            this.numDataBits.AllowNegative = false;
            this.ep_Errors.SetIconAlignment(this.numDataBits, System.Windows.Forms.ErrorIconAlignment.MiddleLeft);
            this.numDataBits.Location = new System.Drawing.Point(112, 194);
            this.numDataBits.Maximum = new decimal(new int[] {
            16,
            0,
            0,
            0});
            this.numDataBits.Minimum = new decimal(new int[] {
            2,
            0,
            0,
            0});
            this.numDataBits.Name = "numDataBits";
            this.numDataBits.Size = new System.Drawing.Size(79, 20);
            this.numDataBits.TabIndex = 1;
            this.numDataBits.Value = new decimal(new int[] {
            2,
            0,
            0,
            0});
            // 
            // cbDataLines
            // 
            this.cbDataLines.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cbDataLines.FormattingEnabled = true;
            this.ep_Errors.SetIconAlignment(this.cbDataLines, System.Windows.Forms.ErrorIconAlignment.MiddleLeft);
            this.cbDataLines.Location = new System.Drawing.Point(112, 168);
            this.cbDataLines.Name = "cbDataLines";
            this.cbDataLines.Size = new System.Drawing.Size(100, 21);
            this.cbDataLines.TabIndex = 1;
            this.cbDataLines.SelectedIndexChanged += new System.EventHandler(this.cbDataLines_SelectedIndexChanged);
            // 
            // lblCalculatedBitRate
            // 
            this.lblCalculatedBitRate.AutoSize = true;
            this.lblCalculatedBitRate.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, ((System.Drawing.FontStyle)((System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Italic))), System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblCalculatedBitRate.Location = new System.Drawing.Point(109, 223);
            this.lblCalculatedBitRate.Name = "lblCalculatedBitRate";
            this.lblCalculatedBitRate.Size = new System.Drawing.Size(41, 13);
            this.lblCalculatedBitRate.TabIndex = 38;
            this.lblCalculatedBitRate.Text = "label1";
            // 
            // m_lblDataLines
            // 
            this.m_lblDataLines.AutoSize = true;
            this.m_lblDataLines.Location = new System.Drawing.Point(39, 171);
            this.m_lblDataLines.Name = "m_lblDataLines";
            this.m_lblDataLines.Size = new System.Drawing.Size(61, 13);
            this.m_lblDataLines.TabIndex = 39;
            this.m_lblDataLines.Text = "Data Lines:";
            // 
            // CySPIMControl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoScroll = true;
            this.AutoValidate = System.Windows.Forms.AutoValidate.EnablePreventFocusChange;
            this.Controls.Add(this.cbDataLines);
            this.Controls.Add(this.m_lblDataLines);
            this.Controls.Add(this.numDataBits);
            this.Controls.Add(this.lblCalculatedBitRate);
            this.Controls.Add(this.cbBitRateHertz);
            this.Controls.Add(this.m_lblBitRate);
            this.Controls.Add(this.cbShiftDir);
            this.Controls.Add(this.lblShiftDir);
            this.Controls.Add(this.pbDrawing);
            this.Controls.Add(this.m_lblNumDataBits);
            this.Controls.Add(this.m_lblspimMode);
            this.Controls.Add(this.cbMode);
            this.Controls.Add(this.numBitRateHertz);
            this.Name = "CySPIMControl";
            this.Size = new System.Drawing.Size(436, 242);
            this.Load += new System.EventHandler(this.CySPIMControl_Load);
            ((System.ComponentModel.ISupportInitialize)(this.pbDrawing)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ep_Errors)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numBitRateHertz)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numDataBits)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox cbMode;
        private System.Windows.Forms.Label m_lblspimMode;
        private System.Windows.Forms.Label m_lblNumDataBits;
        private System.Windows.Forms.PictureBox pbDrawing;
        private System.Windows.Forms.Label lblShiftDir;
        private System.Windows.Forms.ComboBox cbShiftDir;
        private System.Windows.Forms.Label m_lblBitRate;
        private System.Windows.Forms.ComboBox cbBitRateHertz;
        private System.Windows.Forms.ErrorProvider ep_Errors;
        private System.Windows.Forms.Label lblCalculatedBitRate;
        private CyNumericUpDown numBitRateHertz;
        private CyNumericUpDown numDataBits;
        private System.Windows.Forms.ComboBox cbDataLines;
        private System.Windows.Forms.Label m_lblDataLines;
    }
}
