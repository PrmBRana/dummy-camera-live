import os
import sys
from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import inch
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether, HRFlowable
)
from reportlab.pdfgen import canvas

class NumberedCanvas(canvas.Canvas):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_page_decorations(num_pages)
            super().showPage()
        super().save()

    def draw_page_decorations(self, page_count):
        self.saveState()
        self.setFont("Helvetica", 8)
        self.setFillColor(colors.HexColor("#555555"))
        
        # Header (pages 2+)
        if self._pageNumber > 1:
            self.drawString(54, 750, "NuttX STM32WL5 — Dual ADS7953 ADC Driver & OBC_main Technical Report")
            self.setStrokeColor(colors.HexColor("#dcdcdc"))
            self.setLineWidth(0.5)
            self.line(54, 742, 612 - 54, 742)
            
        # Footer (all pages)
        self.setStrokeColor(colors.HexColor("#dcdcdc"))
        self.setLineWidth(0.5)
        self.line(54, 45, 612 - 54, 45)
        self.drawString(54, 32, "Confidential — On-Board Computer (OBC) Subsystem")
        page_text = f"Page {self._pageNumber} of {page_count}"
        self.drawRightString(612 - 54, 32, page_text)
        self.restoreState()

def build_pdf(filename):
    doc = SimpleDocTemplate(
        filename,
        pagesize=letter,
        leftMargin=54,
        rightMargin=54,
        topMargin=54,
        bottomMargin=54
    )

    styles = getSampleStyleSheet()

    # Custom styles
    title_style = ParagraphStyle(
        'DocTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=22,
        leading=26,
        textColor=colors.HexColor("#1a2b49"),
        spaceAfter=6
    )
    
    subtitle_style = ParagraphStyle(
        'DocSubTitle',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=12,
        leading=16,
        textColor=colors.HexColor("#4a5568"),
        spaceAfter=15
    )

    meta_style = ParagraphStyle(
        'DocMeta',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=9,
        leading=13,
        textColor=colors.HexColor("#2b6cb0")
    )

    h1_style = ParagraphStyle(
        'Heading1_Custom',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=14,
        leading=18,
        textColor=colors.HexColor("#1a2b49"),
        spaceBefore=14,
        spaceAfter=8,
        keepWithNext=True
    )

    h2_style = ParagraphStyle(
        'Heading2_Custom',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=11,
        leading=15,
        textColor=colors.HexColor("#2b6cb0"),
        spaceBefore=10,
        spaceAfter=5,
        keepWithNext=True
    )

    body_style = ParagraphStyle(
        'Body_Custom',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=9.5,
        leading=13.5,
        textColor=colors.HexColor("#2d3748"),
        spaceAfter=6
    )

    body_bold = ParagraphStyle(
        'Body_Bold',
        parent=body_style,
        fontName='Helvetica-Bold'
    )

    bullet_style = ParagraphStyle(
        'Bullet_Custom',
        parent=body_style,
        leftIndent=15,
        firstLineIndent=-10,
        spaceAfter=4
    )

    code_style = ParagraphStyle(
        'Code_Custom',
        parent=styles['Normal'],
        fontName='Courier',
        fontSize=8,
        leading=10.5,
        textColor=colors.HexColor("#1a202c")
    )

    table_header_style = ParagraphStyle(
        'THStyle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=8.5,
        leading=11,
        textColor=colors.white
    )

    table_cell_style = ParagraphStyle(
        'TDStyle',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=8,
        leading=10.5,
        textColor=colors.HexColor("#2d3748")
    )

    table_cell_code = ParagraphStyle(
        'TDCode',
        parent=table_cell_style,
        fontName='Courier',
        fontSize=7.5,
        leading=9.5
    )

    elements = []

    # Title Block
    elements.append(Paragraph("Dual TI ADS7953 ADC Driver & OBC_main Application", title_style))
    elements.append(Paragraph("Complete Technical Architecture, Root-Cause Diagnostics & Implementation Report", subtitle_style))
    
    meta_text = "<b>Target Hardware:</b> STM32WL55JC (Dual-Core ARM Cortex-M4 / Cortex-M0+)<br/>" \
                "<b>Operating System:</b> Apache NuttX RTOS v12.13.0<br/>" \
                "<b>Peripheral Bus:</b> SPI2 (Dedicated Multi-Slave Bus) | <b>ADC Devices:</b> 2x TI ADS7953 (32 Channels Total)<br/>" \
                "<b>Target Application:</b> apps/examples/OBC_main | <b>Status:</b> Verified & Operational"
    elements.append(Paragraph(meta_text, meta_style))
    elements.append(Spacer(1, 10))
    elements.append(HRFlowable(width="100%", thickness=1.5, color=colors.HexColor("#1a2b49"), spaceAfter=14))

    # Section 1: Executive Overview
    elements.append(Paragraph("1. Executive Overview & Hardware Setup", h1_style))
    p1 = ("This engineering document provides a comprehensive post-implementation reference for the dual Texas Instruments "
          "<b>ADS7953</b> 16-channel 12-bit SAR ADC subsystem integrated into <b>Apache NuttX RTOS</b> on the <b>STM32WL55JC</b> "
          "microcontroller. The driver provides access to 32 analog telemetry channels (16 channels per chip) across a shared "
          "<b>SPI2</b> bus with independent chip-select lines, exposed to userland as standard POSIX character devices "
          "<code>/dev/adc0</code> (ADC 1) and <code>/dev/adc1</code> (ADC 2).")
    elements.append(Paragraph(p1, body_style))

    # Pinout Table
    pin_data = [
        [Paragraph("Signal Name", table_header_style), Paragraph("MCU Pin", table_header_style), Paragraph("Hardware Role", table_header_style), Paragraph("Configuration Details", table_header_style)],
        [Paragraph("SPI2 SCK", table_cell_bold := ParagraphStyle('B', parent=table_cell_style, fontName='Helvetica-Bold')), Paragraph("PB10", table_cell_code), Paragraph("SPI Serial Clock", table_cell_style), Paragraph("Alternate Function 5 (AF5)", table_cell_style)],
        [Paragraph("SPI2 MISO", table_cell_bold), Paragraph("PB14", table_cell_code), Paragraph("Master In Slave Out (DOUT)", table_cell_style), Paragraph("Alternate Function 5 (AF5)", table_cell_style)],
        [Paragraph("SPI2 MOSI", table_cell_bold), Paragraph("PB15", table_cell_code), Paragraph("Master Out Slave In (DIN)", table_cell_style), Paragraph("Alternate Function 5 (AF5) — Protected from LED conflict", table_cell_style)],
        [Paragraph("CS1 (ADC 1)", table_cell_bold), Paragraph("PB12", table_cell_code), Paragraph("Chip Select 1 (/dev/adc0)", table_cell_style), Paragraph("GPIO Output Push-Pull, Active-LOW (SPIDEV_USER(0))", table_cell_style)],
        [Paragraph("CS2 (ADC 2)", table_cell_bold), Paragraph("PA1", table_cell_code), Paragraph("Chip Select 2 (/dev/adc1)", table_cell_style), Paragraph("GPIO Output Push-Pull, Active-LOW (SPIDEV_USER(1))", table_cell_style)],
    ]
    t_pins = Table(pin_data, colWidths=[1.1*inch, 0.8*inch, 1.8*inch, 3.1*inch])
    t_pins.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#1a2b49")),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#cbd5e0")),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.HexColor("#f7fafc"), colors.white]),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    elements.append(t_pins)
    elements.append(Spacer(1, 10))

    # Section 2: Complete File Modification Inventory
    elements.append(Paragraph("2. File Modification & Creation Inventory", h1_style))
    p2 = ("The table below details all files created, configured, or debugged across the NuttX kernel, board support package, "
          "and application layers to achieve full hardware functionality.")
    elements.append(Paragraph(p2, body_style))

    file_data = [
        [Paragraph("File Path", table_header_style), Paragraph("Type", table_header_style), Paragraph("Key Modifications / Purpose", table_header_style)],
        [Paragraph("nuttx/arch/arm/src/stm32wl5/<br/>stm32wl5_spi.c", table_cell_code), Paragraph("Kernel Driver<br/>(MODIFIED)", table_cell_style), Paragraph("1. Added <code>.lock = NXMUTEX_INITIALIZER</code> to <code>g_spi2s2dev</code> (resolved fatal deadlock).<br/>2. Added timeout counters to <code>spi_readword()</code> and <code>spi_writeword()</code> to guard against hardware hangs.", table_cell_style)],
        [Paragraph("nuttx/boards/arm/stm32wl5/<br/>nucleo-wl55jc/src/stm32_leds.c", table_cell_code), Paragraph("Board BSP<br/>(MODIFIED)", table_cell_style), Paragraph("Guarded <code>GPIO_LED_BLUE</code> (PB15) with <code>#ifndef CONFIG_STM32WL5_SPI2S2</code> so board LED driver does not claim PB15 away from SPI2 MOSI.", table_cell_style)],
        [Paragraph("nuttx/arch/arm/src/stm32wl5/<br/>stm32wl5_rcc.c", table_cell_code), Paragraph("Kernel Driver<br/>(MODIFIED)", table_cell_style), Paragraph("Fixed APB1 peripheral clock gating for SPI2 by enabling <code>RCC_APB1ENR1_SPI2EN</code> when <code>CONFIG_STM32WL5_SPI2S2</code> is defined.", table_cell_style)],
        [Paragraph("nuttx/drivers/analog/<br/>ads7953.c", table_cell_code), Paragraph("Lower-Half Driver<br/>(NEW)", table_cell_style), Paragraph("Implements Texas Instruments ADS7953 driver: SPI Mode 0, manual channel scanning with 2-frame pipeline compensation, and non-blocking registration.", table_cell_style)],
        [Paragraph("nuttx/include/nuttx/analog/<br/>ads7953.h", table_cell_code), Paragraph("Public Header<br/>(NEW)", table_cell_style), Paragraph("Declares <code>ads7953_register()</code>, range control IOCTLs (<code>ANIOC_ADS7953_SET_RANGE</code>), and hardware macros.", table_cell_style)],
        [Paragraph("nuttx/boards/arm/stm32wl5/<br/>nucleo-wl55jc/src/stm32_spi.c", table_cell_code), Paragraph("Board BSP<br/>(MODIFIED)", table_cell_style), Paragraph("Configured <code>stm32wl5_spi2s2select()</code> to map <code>SPIDEV_USER(0)</code> to PB12 and <code>SPIDEV_USER(1)</code> to PA1 with active-LOW logic.", table_cell_style)],
        [Paragraph("nuttx/boards/arm/stm32wl5/<br/>nucleo-wl55jc/src/stm32_boot.c", table_cell_code), Paragraph("Board BSP<br/>(MODIFIED)", table_cell_style), Paragraph("Registers <code>/dev/adc0</code> and <code>/dev/adc1</code> in <code>board_late_initialize()</code> on SPI2 bus with console confirmation prints.", table_cell_style)],
        [Paragraph("nuttx/drivers/analog/<br/>Kconfig, Make.defs, CMakeLists", table_cell_code), Paragraph("Build System<br/>(MODIFIED)", table_cell_style), Paragraph("Integrated <code>CONFIG_ADC_ADS7953</code> into NuttX configuration menu and automated build system.", table_cell_style)],
        [Paragraph("nuttx/.config", table_cell_code), Paragraph("System Config<br/>(MODIFIED)", table_cell_style), Paragraph("Increased <code>CONFIG_ADC_FIFOSIZE</code> from 8 to 32 to prevent FIFO overflow when receiving 16 channels.", table_cell_style)],
        [Paragraph("apps/examples/OBC_main/<br/>main.c", table_cell_code), Paragraph("NSH Application<br/>(MODIFIED)", table_cell_style), Paragraph("Continuous acquisition loop reading both ADC1 and ADC2 every second, printing formatted side-by-side 12-bit raw count tables.", table_cell_style)],
    ]
    t_files = Table(file_data, colWidths=[2.1*inch, 1.2*inch, 3.5*inch])
    t_files.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#1a2b49")),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#cbd5e0")),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#f7fafc")]),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    elements.append(t_files)
    elements.append(PageBreak())

    # Section 3: In-Depth Diagnostic & Root-Cause Analysis
    elements.append(Paragraph("3. In-Depth Root-Cause Diagnostics & Solutions", h1_style))
    elements.append(Paragraph("During bring-up, multiple interlocking issues caused freezes at boot and runtime. Below is the full technical analysis of each bug.", body_style))

    # Bug 1
    elements.append(Paragraph("3.1 Fatal Mutex Deadlock on SPI2 (Runtime Freeze in obc_main)", h2_style))
    b1_desc = ("<b>Symptom:</b> When executing <code>obc_main</code>, the terminal stopped immediately after printing:<br/>"
               "<code>  -> Triggering 16-channel scan on /dev/adc0...</code><br/>"
               "<b>Root Cause:</b> In <code>nuttx/arch/arm/src/stm32wl5/stm32wl5_spi.c</code>, the global SPI device structure for SPI1 "
               "(<code>g_spi1dev</code>) was defined with <code>.lock = NXMUTEX_INITIALIZER,</code>. However, the structure for SPI2 "
               "(<code>g_spi2s2dev</code>) omitted this initializer entirely! In C, uninitialized static struct fields default to zero. "
               "In NuttX, a mutex with count = 0 indicates that the mutex is <i>already locked</i>. "
               "The moment <code>ads7953_readall()</code> invoked <code>SPI_LOCK(spi, true)</code>, the kernel's <code>nxmutex_lock()</code> "
               "found count == 0 and suspended the calling task indefinitely waiting for an unlock that could never happen.<br/>"
               "<b>Resolution:</b> Added <code>.lock = NXMUTEX_INITIALIZER,</code> to <code>g_spi2s2dev</code>.")
    elements.append(Paragraph(b1_desc, body_style))
    elements.append(Spacer(1, 4))

    # Bug 2
    elements.append(Paragraph("3.2 Hardware Pin Conflict on PB15 (SPI2 MOSI vs Blue LED)", h2_style))
    b2_desc = ("<b>Symptom:</b> SPI transactions stalled or received zeros due to corrupted data lines.<br/>"
               "<b>Root Cause:</b> The Nucleo-WL55JC pin mapping header (<code>board.h</code>) assigned PB15 as <code>GPIO_SPI2_MOSI_3</code>. "
               "However, <code>nucleo-wl55jc.h</code> and <code>stm32_leds.c</code> also assigned PB15 as <code>GPIO_LED_BLUE</code>. "
               "During boot, the userled subsystem initialized and called <code>stm32wl5_configgpio(GPIO_LED_BLUE)</code>. "
               "This reconfigured PB15 from Alternate Function 5 (SPI2 MOSI) to a standard GPIO Push-Pull Output, severing the MOSI line "
               "from the SPI2 peripheral.<br/>"
               "<b>Resolution:</b> In <code>stm32_leds.c</code>, conditioned all initialization and drive logic for <code>GPIO_LED_BLUE</code> "
               "under <code>#ifndef CONFIG_STM32WL5_SPI2S2</code>, ensuring PB15 remains solely allocated to SPI2 MOSI.")
    elements.append(Paragraph(b2_desc, body_style))
    elements.append(Spacer(1, 4))

    # Bug 3
    elements.append(Paragraph("3.3 Missing Peripheral Clock in RCC (APB1 Enable)", h2_style))
    b3_desc = ("<b>Symptom:</b> SPI2 peripheral registers were non-responsive.<br/>"
               "<b>Root Cause:</b> In <code>arch/arm/src/stm32wl5/stm32wl5_rcc.c</code>, the clock gate function checked only "
               "<code>#ifdef CONFIG_STM32WL5_SPI2</code>. The dual-core configuration uses the symbol <code>CONFIG_STM32WL5_SPI2S2</code>. "
               "As a result, bit 14 (<code>RCC_APB1ENR1_SPI2EN</code>) remained 0, leaving SPI2 without an active APB1 clock.<br/>"
               "<b>Resolution:</b> Updated condition to <code>#if defined(CONFIG_STM32WL5_SPI2) || defined(CONFIG_STM32WL5_SPI2S2)</code>.")
    elements.append(Paragraph(b3_desc, body_style))
    elements.append(Spacer(1, 4))

    # Bug 4
    elements.append(Paragraph("3.4 Boot Hang during ads7953_register()", h2_style))
    b4_desc = ("<b>Symptom:</b> System froze at boot right after external NOR Flash partition registration before reaching <code>nsh></code>.<br/>"
               "<b>Root Cause:</b> During <code>adc_register()</code>, the upper-half ADC driver immediately invokes <code>ao_reset()</code>. "
               "The initial driver attempted a synchronous SPI reset command across the bus inside <code>ao_reset()</code>. "
               "Executing SPI transfers during early kernel boot before board stabilization caused infinite loops.<br/>"
               "<b>Resolution:</b> Conformed to standard NuttX ADC architecture (e.g. <code>mcp3008.c</code>) by making <code>ao_reset()</code>, "
               "<code>ao_setup()</code>, and <code>ao_rxint()</code> completely non-blocking. Bus access is deferred entirely to runtime requests.")
    elements.append(Paragraph(b4_desc, body_style))
    elements.append(Spacer(1, 4))

    # Bug 5
    elements.append(Paragraph("3.5 ADC FIFO Buffer Overflow", h2_style))
    b5_desc = ("<b>Symptom:</b> Only channels 0 through 6 were received; higher channels were lost.<br/>"
               "<b>Root Cause:</b> The system configuration had <code>CONFIG_ADC_FIFOSIZE=8</code>. A circular buffer of size 8 can only hold "
               "7 elements before its tail catches its head. When the ADS7953 driver enqueued all 16 channels, the upper-half FIFO rejected "
               "channels 7 through 15 with <code>-ENOMEM</code>.<br/>"
               "<b>Resolution:</b> Updated <code>CONFIG_ADC_FIFOSIZE=32</code> in <code>.config</code>.")
    elements.append(Paragraph(b5_desc, body_style))

    # Section 4: ADS7953 Driver Architecture
    elements.append(Spacer(1, 6))
    elements.append(Paragraph("4. ADS7953 Driver Architecture & Pipeline Handling", h1_style))
    p4 = ("The Texas Instruments ADS7953 uses a 16-bit serial frame in <b>SPI Mode 0</b> (CPOL=0, CPHA=0). "
          "In Manual Mode, channel conversions exhibit a <b>2-frame pipeline latency</b>: the channel requested in frame N is sampled "
          "during frame N+1 and its 12-bit conversion result is clocked out on DOUT during frame N+2. "
          "The driver implements an 18-cycle scan sequence to completely drain all 16 channels:")
    elements.append(Paragraph(p4, body_style))

    scan_seq = [
        [Paragraph("Scan Cycle", table_header_style), Paragraph("DIN Command Sent", table_header_style), Paragraph("DOUT Data Returned", table_header_style), Paragraph("Driver Action", table_header_style)],
        [Paragraph("Cycle 0", table_cell_bold), Paragraph("Manual Mode CH00", table_cell_code), Paragraph("Previous chip state", table_cell_style), Paragraph("Discard (pipeline warmup)", table_cell_style)],
        [Paragraph("Cycle 1", table_cell_bold), Paragraph("Manual Mode CH01", table_cell_code), Paragraph("Invalid / warmup", table_cell_style), Paragraph("Discard (pipeline warmup)", table_cell_style)],
        [Paragraph("Cycle 2", table_cell_bold), Paragraph("Manual Mode CH02", table_cell_code), Paragraph("<b>CH00 Sample</b> [12-bit + CH tag]", table_cell_style), Paragraph("Push CH00 to upper-half FIFO", table_cell_style)],
        [Paragraph("Cycle 3 .. 15", table_cell_bold), Paragraph("Manual Mode CH03 .. CH15", table_cell_code), Paragraph("<b>CH01 .. CH13 Samples</b>", table_cell_style), Paragraph("Push CH01 .. CH13 to FIFO", table_cell_style)],
        [Paragraph("Cycle 16", table_cell_bold), Paragraph("Continue (0x0000)", table_cell_code), Paragraph("<b>CH14 Sample</b>", table_cell_style), Paragraph("Push CH14 to FIFO", table_cell_style)],
        [Paragraph("Cycle 17", table_cell_bold), Paragraph("Continue (0x0000)", table_cell_code), Paragraph("<b>CH15 Sample</b>", table_cell_style), Paragraph("Push CH15 to FIFO", table_cell_style)],
    ]
    t_scan = Table(scan_seq, colWidths=[1.1*inch, 2.0*inch, 2.1*inch, 1.6*inch])
    t_scan.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#1a2b49")),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#cbd5e0")),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#f7fafc")]),
        ('TOPPADDING', (0,0), (-1,-1), 3.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3.5),
    ]))
    elements.append(t_scan)
    elements.append(PageBreak())

    # Section 5: OBC_main Application Architecture
    elements.append(Paragraph("5. OBC_main Application & Telemetry Acquisition Flow", h1_style))
    p5 = ("The <code>obc_main</code> application serves as the userland acquisition engine. "
          "It operates as a continuous daemon loop that triggers and reads both ADCs synchronously:")
    elements.append(Paragraph(p5, body_style))

    flow_items = [
        "<b>1. Device Open:</b> Opens <code>/dev/adc0</code> (ADC 1) and <code>/dev/adc1</code> (ADC 2) in read-only mode.",
        "<b>2. Conversion Trigger:</b> Issues <code>ioctl(fd, ANIOC_TRIGGER, 0)</code>, triggering the 18-cycle manual scan in the kernel driver.",
        "<b>3. FIFO Ingestion:</b> Performs a POSIX <code>read()</code> for <code>sizeof(struct adc_msg_s) * 16</code> bytes.",
        "<b>4. Channel Demultiplexing:</b> Maps each <code>adc_msg_s</code> (containing <code>am_channel</code> and 12-bit <code>am_data</code>) into channel arrays.",
        "<b>5. Side-by-Side Formatting:</b> Renders both ADC chips side-by-side in a clean console table.",
        "<b>6. Cadence Control:</b> Sleeps for 1 second before beginning the next acquisition cycle."
    ]
    for item in flow_items:
        elements.append(Paragraph(f"• {item}", bullet_style))

    elements.append(Spacer(1, 8))
    elements.append(Paragraph("Live Console Output Format:", h2_style))

    sample_output = (
        "=====================================================<br/>"
        "  OBC Main: Continuous Raw ADC Reader (TI ADS7953)  <br/>"
        "  Reading 16 Channels from ADC1 (/dev/adc0) & ADC2 (/dev/adc1)<br/>"
        "=====================================================<br/>"
        "&gt;&gt;&gt; Scan #1 &lt;&lt;&lt;<br/>"
        "+---------+--------------------+--------------------+<br/>"
        "| Channel |    ADC 1 (Raw)     |    ADC 2 (Raw)     |<br/>"
        "+---------+--------------------+--------------------+<br/>"
        "|   CH00  |        2048        |        1024        |<br/>"
        "|   CH01  |        1530        |         820        |<br/>"
        "|   CH02  |        4095        |        2048        |<br/>"
        "|   ...   |        ...         |        ...         |<br/>"
        "|   CH15  |        3840        |        1920        |<br/>"
        "+---------+--------------------+--------------------+"
    )
    
    t_box = Table([[Paragraph(sample_output, code_style)]], colWidths=[6.8*inch])
    t_box.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#edf2f7")),
        ('BOX', (0,0), (-1,-1), 1, colors.HexColor("#cbd5e0")),
        ('TOPPADDING', (0,0), (-1,-1), 6),
        ('BOTTOMPADDING', (0,0), (-1,-1), 6),
        ('LEFTPADDING', (0,0), (-1,-1), 8),
        ('RIGHTPADDING', (0,0), (-1,-1), 8),
    ]))
    elements.append(t_box)

    # Section 6: Memory Architecture & Allocation Table
    elements.append(Spacer(1, 10))
    elements.append(Paragraph("6. Memory Architecture & Build-Time Usage Summary", h1_style))
    p_mem = ("The STM32WL55JC contains 256 KB on-chip Flash and 64 KB total SRAM. In this dual-core configuration, "
             "the physical Flash is partitioned into two 128 KB banks: Bank 1 (0x08000000) for CPU1 (ARM Cortex-M4), and Bank 2 "
             "(0x08020000) reserved for CPU2 (ARM Cortex-M0+ Sub-GHz Radio stack). "
             "The build system outputs an optimized memory breakdown with exact bytes, kilobytes, and percentages:")
    elements.append(Paragraph(p_mem, body_style))

    mem_table_data = [
        [Paragraph("Memory Region", table_header_style), Paragraph("Total Capacity", table_header_style), Paragraph("Used Space", table_header_style), Paragraph("Free / Available", table_header_style), Paragraph("Used %", table_header_style), Paragraph("Free %", table_header_style)],
        [Paragraph("CPU1 Flash (App)", table_cell_bold), Paragraph("128.0 KB (131,072 B)", table_cell_style), Paragraph("103.8 KB (106,320 B)", table_cell_style), Paragraph("24.2 KB (24,752 B)", table_cell_style), Paragraph("81.1%", table_cell_style), Paragraph("18.9%", table_cell_style)],
        [Paragraph("CPU2 Flash (M0+)", table_cell_bold), Paragraph("128.0 KB (131,072 B)", table_cell_style), Paragraph("Reserved for Radio", table_cell_style), Paragraph("Reserved (0x08020000)", table_cell_style), Paragraph("--", table_cell_style), Paragraph("--", table_cell_style)],
        [Paragraph("Chip Total Flash", table_cell_bold), Paragraph("256.0 KB (262,144 B)", table_cell_style), Paragraph("103.8 KB (106,320 B)", table_cell_style), Paragraph("152.2 KB (155,824 B)", table_cell_style), Paragraph("40.6%", table_cell_style), Paragraph("59.4%", table_cell_style)],
        [Paragraph("CPU1 SRAM1 (RAM)", table_cell_bold), Paragraph("32.0 KB (32,768 B)", table_cell_style), Paragraph("7.2 KB (7,364 B)", table_cell_style), Paragraph("24.8 KB (25,404 B)", table_cell_style), Paragraph("22.5%", table_cell_style), Paragraph("77.5%", table_cell_style)],
    ]
    t_mem = Table(mem_table_data, colWidths=[1.4*inch, 1.4*inch, 1.3*inch, 1.4*inch, 0.65*inch, 0.65*inch])
    t_mem.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#1a2b49")),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#cbd5e0")),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#f7fafc")]),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    elements.append(t_mem)

    # Section 7: Summary and Maintenance Guidelines
    elements.append(Spacer(1, 10))
    elements.append(Paragraph("7. Maintenance Guidelines & Next Steps", h1_style))
    maint_items = [
        "<b>Calibration & Engineering Units:</b> Raw 12-bit counts (0–4095) can be scaled into engineering units (Voltage, Current, Temperature) using sensor slope/offset coefficients in <code>obc_main</code>.",
        "<b>Range Control:</b> The ADS7953 supports 1X (0 to Vref) and 2X (0 to 2*Vref) input ranges via the <code>ANIOC_ADS7953_SET_RANGE</code> IOCTL.",
        "<b>Shared SPI Bus Integrity:</b> Because the SPI2 bus mutex deadlock is resolved, other SPI2 peripherals can safely coexist with ADC1 and ADC2 using unique chip-select lines.",
        "<b>Flash Memory Footprint:</b> Total CPU1 Flash footprint remains at 103.8 KB (81.1% of the dedicated 128 KB CPU1 bank), safely below the Cortex-M0+ CPU2 boundary at <code>0x08020000</code>."
    ]
    for m in maint_items:
        elements.append(Paragraph(f"• {m}", bullet_style))

    elements.append(Spacer(1, 15))
    elements.append(HRFlowable(width="100%", thickness=1, color=colors.HexColor("#cbd5e0"), spaceAfter=10))
    elements.append(Paragraph("<i>Report generated autonomously by Antigravity IDE for NuttX Embedded Systems Architecture.</i>", table_cell_style))

    doc.build(elements, canvasmaker=NumberedCanvas)
    print(f"PDF generated successfully: {filename}")

if __name__ == "__main__":
    output_pdf = "/home/prem/Desktop/nuttxspace/ADS7953_Dual_ADC_Driver_Documentation.pdf"
    build_pdf(output_pdf)
