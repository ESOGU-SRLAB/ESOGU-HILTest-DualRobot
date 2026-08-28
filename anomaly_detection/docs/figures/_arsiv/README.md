# Kullanılmayan şekiller

Makalenin 5 şekli `figures/` kökündedir (fig1..fig5). Buradakiler önceki
sürümlerin şekilleridir; `makale_uret.py` hiçbirine atıf yapmaz.

| Dosya | Neden burada |
|---|---|
| `fig5_real_cell.png`, `fig6_real_cell.png` | v2 dönemi gerçek hücre izleri; yerini fig5_collision.png aldı |
| `fig5_unseen.png` | `fig_unseen()` çıktısı; main() artık çağırmıyor |
| `fig6_interface.png`, `fig7_interface.png` | arayüz ekran görüntüsü (ikisi birebir aynı); makaleye girmedi |
| `real_system.png` | fig7'nin kaynak ekran görüntüsü — **elle alındı, yeniden üretilemez** |

`fig_real_cell()` ve `fig_unseen()` `makale_figurler.py` içinde duruyor ama
`main()` tarafından çağrılmıyor; gerekirse oradan geri açılabilir.
