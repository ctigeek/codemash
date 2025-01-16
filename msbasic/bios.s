.setcpu "65C02"
.segment "BIOS"

ACIA_DATA   = $5000
ACIA_STATUS = $5001
ACIA_CMD    = $5002
ACIA_CTRL   = $5003

SAVE:
LOAD:
                RTS
MONRDKEY:
CHRIN:
                LDA     ACIA_STATUS    ; Check status.
                AND     #$08           ; Key ready?
                BEQ     @no_keypressed
                LDA     ACIA_DATA      ; Load character.
                JSR     CHROUT
                SEC
                RTS
@no_keypressed:
                CLC
                RTS
MONCOUT:
CHROUT:
                PHA
                STA     ACIA_DATA
                LDA     #$FF
@txdelay:       DEC
                BNE     @txdelay
                PLA
                RTS

.include "wozmon.s"