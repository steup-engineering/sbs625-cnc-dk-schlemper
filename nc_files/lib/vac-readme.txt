o<vac-pod-pick> CALL [<bitmask-left>] [<bitmask-right>]
o<vac-pod-release> CALL

also z.B.
o<vac-pod-pick> CALL [3] [0]
o<vac-pod-release> CALL


vac-pod-pick:
  <bitmask-left>/<bitmask-right>:
    Bitmaske für die Auswahl der Vakuumflächen pro Saugerplatte. Nummerierung ist
    von links nach rechts und von hinten nach vorne, wenn von vorne auf die
    Sägeblattaufnahme geschaut wird:

    1   2   M   1   2
      4     o     4
            t     8
            o
            r

        Sägeblatt


Ablaufsteuerung und Manuelle Bedienung ist auf Maschinenseite realisiert.
