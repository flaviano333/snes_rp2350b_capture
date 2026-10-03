.pio_version 1
.program snes_capture_low
; RP2350 PIO0, GPIO base = 0
;
; GP0      = PHI2
; GP1      = /WR (jmp pin)
; GP2-4    = D0-D2
; GP5      = dummy (unusable)
; GP6-10   = D3-D7
; GP11-19  = A0-A8
;
; in_base = GP2. IN PINS,18 reads GP2..GP19.
; C code removes the GP5 hole and reconstructs D0-D7 + A0-A8.
;
.wrap_target
    wait 1 gpio 0          ; PHI2 high: address bus is stable
    nop [1]                ; brief settling time before sampling /WR
    jmp pin no_write       ; /WR high -> this is not a write cycle

    irq next 0             ; set IRQ0 in PIO1: snapshot high address now
    wait 0 gpio 0          ; data guaranteed valid at PHI2 falling edge

    mov isr, null
    in pins, 18            ; D0-D7 + A0-A7
    push block
    jmp cycle_done

no_write:
    wait 0 gpio 0          ; finish this bus cycle before trying again

cycle_done:
    nop
.wrap
