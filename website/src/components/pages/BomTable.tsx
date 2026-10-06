import bomData from "@content/data/bom.json";
import { Tag } from "@/components/ui/Tag";
import { formatDate } from "./format";
import "./pages.css";

type BomItem = { part: string; qty: number | string; spec: string; notes: string; link?: string; planned?: boolean };
type Bom = { updated: string; groups: { name: string; items: BomItem[] }[] };

const bom = bomData as Bom;

/**
 * Bill of materials from content/data/bom.json: one table per group. Below
 * 40rem each row becomes a stacked card (labels come from data-label).
 */
export function BomTable() {
  return (
    <div className="bom not-prose">
      {bom.groups.map((group) => (
        <div key={group.name} className="bom__group">
          <table className="bom__table">
            <caption className="bom__caption">{group.name}</caption>
            <thead>
              <tr>
                <th scope="col">Part</th>
                <th scope="col">Qty</th>
                <th scope="col">Spec</th>
                <th scope="col">Notes</th>
              </tr>
            </thead>
            <tbody>
              {group.items.map((item) => (
                <tr key={item.part}>
                  <th scope="row" className="bom__part">
                    {item.link ? <a href={item.link}>{item.part}</a> : item.part}
                    {item.planned && (
                      <>
                        {" "}
                        <Tag dot>Planned</Tag>
                      </>
                    )}
                  </th>
                  <td data-label="Qty" className="bom__qty">
                    {item.qty}
                  </td>
                  <td data-label="Spec">
                    {item.spec}
                  </td>
                  <td data-label="Notes" className="bom__notes">
                    {item.notes}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      ))}
      <p className="caption bom__updated">
        Updated <time dateTime={bom.updated}>{formatDate(bom.updated)}</time>
      </p>
    </div>
  );
}
