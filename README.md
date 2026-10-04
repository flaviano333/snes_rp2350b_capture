# SNES RetroAchievements Bridge

Projeto experimental para usar **RetroAchievements com um Super Nintendo real**.

O sistema utiliza um **RP2350B** conectado ao barramento do SNES para capturar a memória WRAM do console em tempo real.  
No computador, um bridge em Python disponibiliza esses dados através de uma interface compatível com **usb2snes / RA2Snes**, permitindo que o RetroAchievements acompanhe o estado do jogo rodando no hardware original.

## Como funciona

O fluxo básico é:

SNES físico → RP2350B → USB → Bridge Python → RA2Snes → RetroAchievements

O RP2350B monitora os acessos à memória do SNES e mantém um espelho da WRAM.  
O bridge envia esses dados para o RetroAchievements quando são solicitados.

## Recursos

- Captura da WRAM do SNES em tempo real
- Suporte a `$7E` e `$7F`
- Uso de `/WRAMSEL` para validar acessos à WRAM
- Captura de escritas
- Correção através de leituras (`READ-REPAIR`)
- Snapshots consistentes para o RA2Snes
- Comunicação USB serial com o RP2350B
- Servidor compatível com usb2snes
- Identificação de ROM para o RetroAchievements
- AUTO-ROM em desenvolvimento

## Hardware

O projeto atualmente utiliza:

- Super Nintendo
- RP2350B
- Conexão ao barramento de endereço e dados do SNES
- `/RD`
- `/WR`
- `/ROMSEL`
- `/WRAMSEL`
- USB conectado ao computador

## Software

O bridge foi desenvolvido em **Python**.

O firmware do RP2350B utiliza o **Pico SDK**, PIO e DMA para acompanhar o barramento do SNES.

## Estado do projeto

O projeto ainda está em desenvolvimento.

A captura de conquistas através da WRAM física já está funcional e foi testada com jogos reais no SNES.

A identificação automática do cartucho através das leituras da ROM ainda está sendo desenvolvida.

## ROMs

ROMs comerciais **não são incluídas neste repositório**.

Caso uma ROM de referência seja necessária para identificação pelo RetroAchievements, o usuário deve fornecer sua própria cópia.

## Aviso

Este é um projeto experimental, feito com auxilio de inteligência artificial e pode exigir ajustes dependendo da revisão do console, cartucho e configuração utilizada.
