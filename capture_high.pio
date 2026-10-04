.pio_version 1
.program snes_capture_high
; RP2350 PIO1, GPIO base = 16
;
; GP21     = A9
; GP22     = dummy / unused
; GP23-25  = A11-A13
; GP26     = dummy / unusable
; GP27-34  = A14-A21
; GP35     = /RD dummy in sampled window
; GP36-37  = A22-A23
; GP38     = /ROMSEL dummy in sampled window
; GP39     = /WRAMSEL (active-low qualifier; consumed by C)
; GP40     = A10
;
; in_base = GP21. IN PINS,20 reads GP21..GP40.
; C reconstructs A9..A23 and takes A10 specifically from GP40.
;
.wrap_target
    wait 1 irq 0           ; event sent internally by PIO0, no jumper needed
    mov isr, null
    in pins, 20            ; A8,A9,dummy,A10..A23
    push block
.wrap
